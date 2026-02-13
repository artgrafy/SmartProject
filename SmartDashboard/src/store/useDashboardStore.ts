import { create } from 'zustand';
import { SensorData, DeviceInfo } from '@/types/sensor';
import { calculateBPM, calculateStressIndex } from '@/utils/analysis';

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

            return {
                latestData: data,
                history: newHistory,
                device: { ...state.device, lastUpdate: Date.now() },
                // 새로운 호흡 정점(Peak)이 발견되었을 때만 BPM 수치를 업데이트하여 화면 떨림 방지
                calculatedBPM: newBPM !== -1 ? newBPM : state.calculatedBPM,
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
