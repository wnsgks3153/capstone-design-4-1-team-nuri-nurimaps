#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// 3D 좌표를 표현하는 구조체
typedef struct {
    double x;
    double y;
    double z;
} Point3D;

// 앵커 정보를 표현하는 구조체
typedef struct {
    int id;              // 앵커 ID
    double distance;     // 태그와의 거리 (cm)
    Point3D position;    // 앵커의 위치 (cm) - 3D 좌표
} Anchor3D;

// 앵커 위치 데이터베이스 (실제 앵커 위치, 시스템 설정에서 로드)
void load_anchor_positions(Point3D anchor_positions[], int max_anchors) {
    // 여기서는 예시 데이터를 사용. 실제로는 설정 파일이나 DB에서 로드해야 함
    // 앵커의 z좌표(높이)를 포함하여 설정 - 앵커는 천장이나 벽 상단에 설치됨을 가정
    anchor_positions[0] = (Point3D){0.0, 0.0, 250.0};      // ID: 1의 위치
    anchor_positions[1] = (Point3D){300.0, 0.0, 250.0};    // ID: 2의 위치
    anchor_positions[2] = (Point3D){150.0, 250.0, 250.0};  // ID: 3의 위치
    anchor_positions[3] = (Point3D){0.0, 250.0, 250.0};    // ID: 4의 위치
    // 필요한 만큼 추가 가능
}

// 문자열에서 앵커 ID와 거리 정보를 파싱하는 함수
int parse_anchor_distances(const char* input, Anchor3D anchors[], int max_anchors) {
    char buffer[256];
    strncpy(buffer, input, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char* token = strtok(buffer, " ");
    int count = 0;
    
    // 앵커 위치 로드
    Point3D anchor_positions[10]; // 최대 10개의 앵커 지원
    load_anchor_positions(anchor_positions, 10);
    
    while (token != NULL && count < max_anchors) {
        int id;
        // ID 파싱 (형식: "02:")
        if (sscanf(token, "%d:", &id) == 1) {
            // 다음 토큰 (거리)
            token = strtok(NULL, " ");
            if (token != NULL) {
                double distance;
                if (sscanf(token, "%lf", &distance) == 1) {
                    // 앵커 정보 저장
                    anchors[count].id = id;
                    anchors[count].distance = distance;
                    
                    // 앵커 ID에 해당하는 위치 정보 로드 (ID는 1부터 시작)
                    if (id >= 1 && id <= 10) {
                        anchors[count].position = anchor_positions[id-1];
                    } else {
                        // 기본 위치 설정
                        anchors[count].position = (Point3D){0.0, 0.0, 0.0};
                    }
                    
                    count++;
                }
            }
        }
        token = strtok(NULL, " ");
    }
    
    return count;
}

// 속도 최적화된 위치 계산 함수
Point3D fast_position_calculation(Anchor3D anchors[], int count, double tag_height) {
    Point3D result = {0.0, 0.0, tag_height}; // 초기 위치, z는 태그 높이로 고정
    
    if (count == 2) {
        // 두 앵커의 중심점을 초기 추정점으로 사용
        result.x = (anchors[0].position.x + anchors[1].position.x) / 2;
        result.y = (anchors[0].position.y + anchors[1].position.y) / 2;
        
        // 높이 보정된 2D 거리 계산
        double planar_distances[2];
        for (int i = 0; i < 2; i++) {
            double height_diff = anchors[i].position.z - tag_height;
            double height_diff_squared = height_diff * height_diff;
            double measured_dist_squared = anchors[i].distance * anchors[i].distance;
            
            if (measured_dist_squared > height_diff_squared) {
                planar_distances[i] = sqrt(measured_dist_squared - height_diff_squared);
            } else {
                planar_distances[i] = 1.0; // 최소값 설정
            }
        }
        
        // 두 원의 교점 계산 시도
        double d = sqrt(pow(anchors[1].position.x - anchors[0].position.x, 2) + 
                       pow(anchors[1].position.y - anchors[0].position.y, 2));
        
        // 두 원이 교차하는지 확인
        if (d <= planar_distances[0] + planar_distances[1] && 
            d >= fabs(planar_distances[0] - planar_distances[1])) {
            
            // 두 원의 교점 계산 로직 (고급 수학 계산)
            // 여기서는 생략하지만, 실제로는 두 교점을 찾고 
            // 추가 정보(이전 위치 등)를 이용해 하나를 선택할 수 있음
            
            printf("두 앵커의 교점을 계산하여 위치 추정이 가능합니다.\n");
        } else {
            printf("두 앵커로부터의 거리 정보가 일관되지 않습니다. 평균 위치를 사용합니다.\n");
        }
        
        return result;
    }

    // 2D 거리 계산 (미리 계산하여 저장)
    double planar_distances[10];
    for (int i = 0; i < count; i++) {
        double height_diff = anchors[i].position.z - tag_height;
        double height_diff_squared = height_diff * height_diff;
        double measured_dist_squared = anchors[i].distance * anchors[i].distance;
        
        // 피타고라스 정리: d^2 = dx^2 + dy^2 + dz^2
        // 따라서 dx^2 + dy^2 = d^2 - dz^2
        if (measured_dist_squared > height_diff_squared) {
            planar_distances[i] = sqrt(measured_dist_squared - height_diff_squared);
        } else {
            // 높이 차이가 측정 거리보다 큰 경우 (비현실적인 상황)
            planar_distances[i] = 1.0; // 최소값 설정
        }
    }
    
    // 초기 위치 추정 개선 (삼변측량 사용)
    if (count >= 3) {
        // 삼변측량으로 초기 위치 계산
        double A[2][2];
        double b[2];
        
        A[0][0] = 2.0 * (anchors[1].position.x - anchors[0].position.x);
        A[0][1] = 2.0 * (anchors[1].position.y - anchors[0].position.y);
        A[1][0] = 2.0 * (anchors[2].position.x - anchors[0].position.x);
        A[1][1] = 2.0 * (anchors[2].position.y - anchors[0].position.y);
        
        b[0] = pow(planar_distances[0], 2) - pow(planar_distances[1], 2) + 
               pow(anchors[1].position.x, 2) - pow(anchors[0].position.x, 2) + 
               pow(anchors[1].position.y, 2) - pow(anchors[0].position.y, 2);
        b[1] = pow(planar_distances[0], 2) - pow(planar_distances[2], 2) + 
               pow(anchors[2].position.x, 2) - pow(anchors[0].position.x, 2) + 
               pow(anchors[2].position.y, 2) - pow(anchors[0].position.y, 2);
        
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (fabs(det) > 1e-6) {
            result.x = (A[1][1] * b[0] - A[0][1] * b[1]) / det;
            result.y = (A[0][0] * b[1] - A[1][0] * b[0]) / det;
        } else {
            // 행렬식이 0에 가까우면 앵커 중심점 사용
            for (int i = 0; i < count; i++) {
                result.x += anchors[i].position.x;
                result.y += anchors[i].position.y;
            }
            result.x /= count;
            result.y /= count;
        }
    } else {
        // 중심점으로 초기화
        for (int i = 0; i < count; i++) {
            result.x += anchors[i].position.x;
            result.y += anchors[i].position.y;
        }
        result.x /= count;
        result.y /= count;
    }
    
    // 최적화 파라미터 (속도 향상을 위해 조정)
    const int MAX_ITERATIONS = 50;   // 줄인 반복 횟수
    const double CONVERGENCE_THRESHOLD = 0.1;  // 증가된 수렴 임계값
    const double ACCEPTABLE_ERROR = 1.0;  // 수용 가능한 오차
    double learning_rate = 1.0;  // 증가된 학습률
    double prev_error = 1e10;
    
    // 반복적 개선 (최적화된 버전)
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        double total_error = 0.0;
        double gradient_x = 0.0;
        double gradient_y = 0.0;
        
        // 각 앵커에 대한 그래디언트 계산 (최적화)
        for (int i = 0; i < count; i++) {
            double dx = result.x - anchors[i].position.x;
            double dy = result.y - anchors[i].position.y;
            
            double dx2 = dx * dx;
            double dy2 = dy * dy;
            
            double calc_2d_dist_squared = dx2 + dy2;
            double calc_2d_dist = sqrt(calc_2d_dist_squared);
            
            if (calc_2d_dist < 1e-6) calc_2d_dist = 1e-6;
            
            double inv_dist = 1.0 / calc_2d_dist; // 나눗셈 대신 곱셈 사용
            
            double error = calc_2d_dist - planar_distances[i];
            total_error += error * error;
            
            double scale = 2.0 * error * inv_dist;
            gradient_x += dx * scale;
            gradient_y += dy * scale;
        }
        
        double rmse = sqrt(total_error / count);
        
        // 조기 종료 조건
        if (rmse < ACCEPTABLE_ERROR) {
            break;
        }
        
        double improvement = prev_error - rmse;
        if (improvement < CONVERGENCE_THRESHOLD) {
            break;
        }
        
        // 위치 업데이트
        double step_x = learning_rate * gradient_x;
        double step_y = learning_rate * gradient_y;
        
        // 스텝 크기 제한
        double step_mag = sqrt(step_x*step_x + step_y*step_y);
        if (step_mag > 20.0) {
            double scale = 20.0 / step_mag;
            step_x *= scale;
            step_y *= scale;
        }
        
        result.x -= step_x;
        result.y -= step_y;
        
        prev_error = rmse;
    }
    
    return result;
}

int main() {
    // 태그로부터 받은 거리 정보 문자열 (4개 앵커)
    const char* input = "01: 210.0 02: 250.0 03: 190.0 04: 230.0";
    
    // 앵커 정보를 저장할 배열
    Anchor3D anchors[10];
    int count;
    
    // 고정된 태그 높이 사용 (평균값)
    double tag_height = 110.0;
    
    // 문자열 파싱
    count = parse_anchor_distances(input, anchors, 10);
    
    // 파싱된 앵커 정보 출력 (디버깅 용도, 필요시 제거)
    printf("파싱된 앵커 정보:\n");
    for (int i = 0; i < count; i++) {
        printf("앵커 ID: %d, 거리: %.2f cm, 위치: (%.2f, %.2f, %.2f)\n", 
               anchors[i].id, anchors[i].distance, 
               anchors[i].position.x, anchors[i].position.y, anchors[i].position.z);
    }
    
    // 빠른 위치 계산 함수 호출
    Point3D tag_position = fast_position_calculation(anchors, count, tag_height);
    
    // 결과 출력 (최소화)
    printf("\n태그 위치: (%.2f, %.2f, %.2f) cm\n", 
           tag_position.x, tag_position.y, tag_position.z);
    
    return 0;
}