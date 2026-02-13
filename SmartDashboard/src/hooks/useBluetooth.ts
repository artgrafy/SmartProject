import { useCallback, useRef } from 'react';
import { useDashboardStore } from '@/store/useDashboardStore';
import { SensorData } from '@/types/sensor';

const SERVICE_UUID = "e9321e36-6211-11ee-8c99-0242ac120002";
const MANUFACTURER_ID = 0xFFFF;

export const useBluetooth = () => {
    const setLatestData = useDashboardStore((state) => state.setLatestData);
    const setDeviceStatus = useDashboardStore((state) => state.setDeviceStatus);
    const deviceRef = useRef<any>(null); // Type 'any' to avoid Bluetooth build errors for now
    const demoIntervalRef = useRef<NodeJS.Timeout | null>(null);

    const parseData = (dataView: DataView): SensorData => {
        return {
            dataType: dataView.getInt16(0, true),
            temp: dataView.getInt16(2, true) / 10.0,
            humidity: dataView.getInt16(4, true) / 10.0,
            pressure: dataView.getInt16(6, true) / 10.0,
            accelX: dataView.getInt16(8, true) / 10.0,
            accelY: dataView.getInt16(10, true) / 10.0,
            accelZ: dataView.getInt16(12, true) / 10.0,
            angleX: dataView.getInt16(14, true) / 10.0,
            angleY: dataView.getInt16(16, true) / 10.0,
            angleZ: dataView.getInt16(18, true) / 10.0,
            resistance: dataView.getInt16(20, true) / 100.0,
            noise: dataView.getInt16(22, true) / 10.0,
            timestamp: Date.now(),
        };
    };

    const startDemo = useCallback(() => {
        if (demoIntervalRef.current) return;

        setDeviceStatus(true, 'demo-device', 'Demo Simulator');

        let angle = 0;
        demoIntervalRef.current = setInterval(() => {
            const baseResistance = 10.5;
            const amplitude = 0.5;
            const resistance = baseResistance + Math.sin(angle) * amplitude + (Math.random() * 0.1);

            const demoData: SensorData = {
                dataType: 2,
                temp: 24.5 + (Math.random() * 0.5),
                humidity: 45.0 + (Math.random() * 2),
                pressure: 1013.2,
                accelX: 0.1, accelY: 0.0, accelZ: 9.8,
                angleX: Math.sin(angle * 0.5) * 5,
                angleY: Math.cos(angle * 0.5) * 5,
                angleZ: 0,
                resistance: parseFloat(resistance.toFixed(2)),
                noise: 35.0 + (Math.random() * 5),
                timestamp: Date.now(),
            };

            setLatestData(demoData);
            angle += 0.2;
        }, 200);
    }, [setLatestData, setDeviceStatus]);

    const stopDemo = useCallback(() => {
        if (demoIntervalRef.current) {
            clearInterval(demoIntervalRef.current);
            demoIntervalRef.current = null;
        }
    }, []);

    const connect = useCallback(async () => {
        stopDemo();
        try {
            // @ts-ignore
            const device = await navigator.bluetooth.requestDevice({
                filters: [
                    { services: [SERVICE_UUID] },
                    { namePrefix: 'CONTROL' },
                    { namePrefix: 'SENSE' }
                ],
                optionalServices: [SERVICE_UUID]
            });

            deviceRef.current = device;
            setDeviceStatus(true, device.id, device.name);

            // @ts-ignore
            await device.watchAdvertisements();

            device.addEventListener('advertisementreceived', (event: any) => {
                const mData = event.manufacturerData.get(MANUFACTURER_ID);
                if (mData) {
                    const parsed = parseData(mData);
                    setLatestData(parsed);
                }
            });

            device.addEventListener('gattserverdisconnected', () => {
                setDeviceStatus(false);
            });

        } catch (error) {
            console.error('Bluetooth Connection Error:', error);
            setDeviceStatus(false);
        }
    }, [setLatestData, setDeviceStatus, stopDemo]);

    const disconnect = useCallback(() => {
        stopDemo();
        if (deviceRef.current?.gatt?.connected) {
            deviceRef.current.gatt.disconnect();
        }
        setDeviceStatus(false);
    }, [setDeviceStatus, stopDemo]);

    return { connect, disconnect, startDemo, stopDemo };
};
