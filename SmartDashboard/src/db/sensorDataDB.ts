import Dexie, { Table } from 'dexie';
import { SensorData } from '@/types/sensor';

/**
 * Smart Dashboard용 오프라인 데이터베이스 관리 (IndexedDB)
 * Dexie.js를 사용하여 고성능 센서 로그 저장을 구현합니다.
 */
export class SensorDataDatabase extends Dexie {
  // 모든 센서의 실시간 기록 (200ms 주기의 Raw 데이터 포함)
  sensorLogs!: Table<SensorData>;

  constructor() {
    super('SmartDashboardDB');
    
    // 테이블 스키마 버전 1 정의
    // timestamp를 기본키로 사용하며, 분석을 위해 dataType을 인덱스로 등록합니다.
    this.version(1).stores({
      sensorLogs: 'timestamp, dataType' 
    });
  }

  // 헬퍼 메소드: 최근 데이터 조회 (예: 차트 복구용)
  async getRecentLogs(limit: number = 200) {
    return await this.sensorLogs
      .orderBy('timestamp')
      .reverse()
      .limit(limit)
      .toArray();
  }

  // 헬퍼 메소드: 특정 기간 데이터 삭제 (로그 관리용)
  async clearOldLogs(days: number = 7) {
    const cutoff = Date.now() - (days * 24 * 60 * 60 * 1000);
    return await this.sensorLogs
      .where('timestamp')
      .below(cutoff)
      .delete();
  }
}

// 싱글톤 인스턴스 수출
export const db = new SensorDataDatabase();
