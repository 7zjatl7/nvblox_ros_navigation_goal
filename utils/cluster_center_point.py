from sklearn.cluster import KMeans
import numpy as np

# 100개의 2차원 랜덤 데이터 생성
data = np.random.randint(100, size=(100, 2))  # 각 데이터 포인트는 2차원
outliers = np.random.randint(200,300, size=(10, 2)) # 10개 이상치 추가
data = np.vstack([data, outliers])

# Z-점수 계산 함수 (2차원 데이터용)
def calculate_z_scores_2d(data):
    means = np.mean(data, axis=0)
    stds = np.std(data, axis=0)
    z_scores = (data - means) / stds
    return z_scores

# 이상치 제거 함수 (2차원 데이터용)
def remove_outliers_2d(data, threshold=2):
    z_scores = calculate_z_scores_2d(data)
    return np.array([data[i] for i in range(len(data)) if np.all(np.abs(z_scores[i]) <= threshold)])

# 이상치 제거 실행 (2차원 데이터용)
filtered_data = remove_outliers_2d(data)

# K-평균 클러스터링 실행
kmeans = KMeans(n_clusters=1)  # 클러스터의 수를 3으로 설정
kmeans.fit(filtered_data)
labels = kmeans.labels_

# 클러스터 중심점 표시
centers = kmeans.cluster_centers_
print(centers)
# # 파일 경로 지정
# file_path = '../maps/goals.txt'
# # 배열을 텍스트 파일에 추가 모드로 쓰기
# with open(file_path, 'a') as file:
#     for row in centers:
#         row_str = ' '.join(map(str, row))  # 각 행을 문자열로 변환하여 공백으로 구분하여 합침
#         file.write(row_str + '\n')

        