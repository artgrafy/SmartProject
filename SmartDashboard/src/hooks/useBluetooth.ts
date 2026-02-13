import { useCallback, useRef } from 'react';
import { useDashboardStore } from '@/store/useDashboardStore';
import { SensorData } from '@/types/sensor';

const SERVICE_UUID = "e9321e36-6211-11ee-8c99-0242ac120002";
const MANUFACTURER_ID = 0xFFFF;

export const useBluetooth = () => {
    const setLatestData = useDashboardStore((state) => state.setLatestData);
    const setDeviceStatus = useDashboardStore((state) => state.setDeviceStatus);
    const deviceRef = useRef<BluetoothDevice | null>(null);

    const parseData = (dataView: DataView): SensorData => {
        // Little Endian parsing (Arduino is Little Endian)
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

    const connect = useCallback(async () => {
        try {
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

            // Bluetooth Advertisement 수신 시작 (Experimental API)
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
    }, [setLatestData, setDeviceStatus]);

    const disconnect = useCallback(() => {
        if (deviceRef.current?.gatt?.connected) {
            deviceRef.current.gatt.disconnect();
        }
        setDeviceStatus(false);
    }, [setDeviceStatus]);

    return { connect, disconnect };
};
