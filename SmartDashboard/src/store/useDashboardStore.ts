import { create } from 'zustand';
import { SensorData, DeviceInfo } from '@/types/sensor';
import { calculateBPM, calculateStressIndex } from '@/utils/analysis';
import { db } from '@/db/sensorDataDB';

interface DashboardStore {
    latestData: SensorData | null;
    history: SensorData[];
    device: DeviceInfo;
    calculatedBPM: number;
    stressIndex: number;
    setLatestData: (data: SensorData) => void;
    setDeviceStatus: (connected: boolean, id?: string, name?: string) => void;
    clearHistory: () => void;
}

export const useDashboardStore = create<DashboardStore>((set) => ({
    latestData: null,
    history: [],
    device: {
        name: 'SmartTransmitter',
        id: '',
        connected: false,
        lastUpdate: 0,
    },
    calculatedBPM: 0,
    stressIndex: 0,
    setLatestData: (data) =>
        set((state) => {
            // 200ms 주기로 데이터가 들어오므로, 최근 40초 정도의 데이터(200개)를 유지합니다.
            const newHistory = [data, ...state.history].slice(0, 200);

            // 실시간 분석 엔진 가동
            const newBPM = calculateBPM(newHistory);
            const newStress = calculateStressIndex(newHistory);
            const finalBPM = newBPM !== -1 ? newBPM : state.calculatedBPM;

            // IndexedDB에 실시간 데이터 영구 저장 (비동기)
            db.sensorLogs.add({
                ...data,
                calculatedBPM: finalBPM,
                stressIndex: newStress
            }).catch(err => console.error("데이터베이스 저장 에러:", err));

            return {
                latestData: data,
                history: newHistory,
                device: { ...state.device, lastUpdate: Date.now() },
                calculatedBPM: finalBPM,
                stressIndex: newStress
            };
        }),
    setDeviceStatus: (connected, id, name) =>
        set((state) => ({
            device: {
                ...state.device,
                connected,
                id: id ?? state.device.id,
                name: name ?? state.device.name,
            },
        })),
    clearHistory: () => set({ history: [], calculatedBPM: 0, stressIndex: 0 }),
}));
