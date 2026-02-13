// 핀 설정 및 기본 상수
const int SENSOR_PIN = A0;
const int SAMPLE_INTERVAL = 100;    // 샘플링 간격 (ms)
const float R1 = 10000.0;          // 기준 저항 10kΩ

// 호흡 감지 매개변수
const float THRESHOLD = 0.5;        // 변화 감지를 위한 임계값 (kΩ)
const int MIN_INTERVAL = 1000;      // 최소 호흡 간격 (1초)
const int MAX_INTERVAL = 10000;     // 최대 호흡 간격 (10초)
const int WINDOW_SIZE = 3;          // 이동 평균 윈도우 크기

// 이동 평균 필터 버퍼
float filterBuffer[WINDOW_SIZE];
int filterIndex = 0;

// 호흡 상태 변수
float maxResistance;               // 현재 고점
float minResistance;               // 현재 저점
unsigned long lastExtremeTime = 0;  // 마지막 극값 시간
bool findingMax = true;            // true: 고점 탐색 중, false: 저점 탐색 중
float lastValidResistance;         // 마지막으로 감지된 유효한 저항값

// 호흡 주기 변수
unsigned long lastBreathTime = 0;   // 마지막 호흡 시간
float currentBreathRate = 0;        // 현재 호흡수

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  
  // 초기값 설정을 위해 여러 번 읽기
  float sum = 0;
  for(int i = 0; i < 10; i++) {
    int rawValue = analogRead(SENSOR_PIN);
    sum += calculateResistance(rawValue);
    delay(10);
  }
  float initialResistance = sum / 10;
  
  // 버퍼 및 변수 초기화
  for(int i = 0; i < WINDOW_SIZE; i++) {
    filterBuffer[i] = initialResistance;
  }
  maxResistance = initialResistance;
  minResistance = initialResistance;
  lastValidResistance = initialResistance;
}

float calculateResistance(int rawValue) {
  if(rawValue < 20) return 999999.0; // 노이즈 필터링
  return R1 * (1023.0/rawValue - 1.0) / 1000.0;
}

float movingAverage(float newValue) {
  filterBuffer[filterIndex] = newValue;
  filterIndex = (filterIndex + 1) % WINDOW_SIZE;
  
  float sum = 0;
  for(int i = 0; i < WINDOW_SIZE; i++) {
    sum += filterBuffer[i];
  }
  return sum / WINDOW_SIZE;
}

// 호흡수 계산 및 업데이트
void updateBreathRate(unsigned long currentTime) {
  if(lastBreathTime == 0) {
    lastBreathTime = currentTime;
    return;
  }
  
  unsigned long interval = currentTime - lastBreathTime;
  if(interval >= MIN_INTERVAL && interval <= MAX_INTERVAL) {
    currentBreathRate = 60000.0 / interval;
    lastBreathTime = currentTime;
    
    Serial.print("호흡 간격: ");
    Serial.print(interval);
    Serial.print("ms, 호흡수: ");
    Serial.print(currentBreathRate);
    Serial.println(" 회/분");
  }
}

void loop() {
  // 센서 값 읽기 및 저항 계산
  int rawValue = analogRead(SENSOR_PIN);
  float resistance = calculateResistance(rawValue);
  float filteredResistance = movingAverage(resistance);
  
  // 현재 시간
  unsigned long currentTime = millis();
  
  // 디버깅을 위한 현재 상태 출력
  Serial.print("현재: ");
  Serial.print(filteredResistance);
  Serial.print(" kΩ, ");
  Serial.print(findingMax ? "고점" : "저점");
  Serial.print(" 탐색 중 (");
  Serial.print(findingMax ? maxResistance : minResistance);
  Serial.println(" kΩ)");
  
  if(findingMax) {
    // 고점 탐색 중
    if(filteredResistance > maxResistance) {
      maxResistance = filteredResistance;
    }
    // 고점 감지 조건: 현재값이 최대값보다 THRESHOLD 이상 감소
    else if(filteredResistance < maxResistance - THRESHOLD) {
      Serial.println("------------------------");
      Serial.print("고점 감지! 저항: ");
      Serial.print(maxResistance);
      Serial.println(" kΩ");
      
      updateBreathRate(currentTime);
      Serial.println("------------------------");
      
      // 상태 전환
      findingMax = false;
      minResistance = filteredResistance;  // 새로운 저점 탐색 시작
      lastValidResistance = maxResistance;
    }
  }
  else {
    // 저점 탐색 중
    if(filteredResistance < minResistance) {
      minResistance = filteredResistance;
    }
    // 저점 감지 조건: 현재값이 최소값보다 THRESHOLD 이상 증가
    else if(filteredResistance > minResistance + THRESHOLD) {
      Serial.println("------------------------");
      Serial.print("저점 감지! 저항: ");
      Serial.print(minResistance);
      Serial.println(" kΩ");
      
      updateBreathRate(currentTime);
      Serial.println("------------------------");
      
      // 상태 전환
      findingMax = true;
      maxResistance = filteredResistance;  // 새로운 고점 탐색 시작
      lastValidResistance = minResistance;
    }
  }
  
  delay(SAMPLE_INTERVAL);
}