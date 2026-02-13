/**
 * 호흡 데이터 분석 및 피크 검출 유틸리티
 */

let lastPeakTime = 0;
const MIN_BREATH_INTERVAL = 1500; // 최소 호흡 간격 (ms) - 40bpm 제한
const MAX_BREATH_INTERVAL = 10000; // 최대 호흡 간격 (ms) - 6bpm 제한

/**
 * 단순 피크 검출 알고리즘 (BPM 계산용)
 * @param history 수신된 센서 데이터 배열
 * @returns 계산된 BPM (분당 호흡수)
 */
export const calculateBPM = (history: { resistance: number, timestamp: number }[]): number => {
    if (history.length < 10) return 0;

    // 최근 5개 데이터로 추세 파악 (간단한 로컬 맥시마 검출)
    const current = history[0]; // 가장 최근
    const prev = history[1];
    const prev2 = history[2];

    // 고점(Peak) 판단 조건: 이전보다 높고, 그 다음 데이터보다 높을 때
    // 실제로는 더 정교한 임계값 처리가 필요함
    if (prev.resistance > prev2.resistance && prev.resistance > current.resistance) {
        const currentTime = prev.timestamp;

        // 중복 감지 방지 (최소 간격 체크)
        if (currentTime - lastPeakTime > MIN_BREATH_INTERVAL) {
            const interval = currentTime - lastPeakTime;
            lastPeakTime = currentTime;

            // 유효 범위 내의 호흡인 경우에만 BPM 계산
            if (interval < MAX_BREATH_INTERVAL) {
                return Math.round(60000 / interval);
            }
        }
    }

    return -1; // 새로운 피크가 발견되지 않음
};

/**
 * 스트레스 지수 분석 (AI 로직 초안)
 * @param history 최근 호흡 데이터
 * @returns 0-100 사이의 스트레스 지수
 */
export const calculateStressIndex = (history: { resistance: number, timestamp: number }[]): number => {
    if (history.length < 50) return 0;

    // 1. 호흡 변이도(Variability) 분석
    // 2. 호흡 깊이(Amplitude) 분석 
    // 여기서는 프로토타입으로 무작위성과 호흡수를 조합한 가상 지수 반환
    // 추후 실제 AI 모델(TensorFlow.js)로 교체될 영역입니다.
    return Math.floor(Math.random() * 20) + 40; // 임시: 40~60 사이 유지
};
