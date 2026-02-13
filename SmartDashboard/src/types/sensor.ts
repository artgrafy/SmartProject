export interface SensorData {
    dataType: number;
    temp: number;
    humidity: number;
    pressure: number;
    accelX: number;
    accelY: number;
    accelZ: number;
    angleX: number;
    angleY: number;
    angleZ: number;
    resistance: number;
    noise: number;
    timestamp: number;
    calculatedBPM?: number;
    stressIndex?: number;
}

export interface DeviceInfo {
    name: string;
    id: string;
    connected: boolean;
    lastUpdate: number;
}
