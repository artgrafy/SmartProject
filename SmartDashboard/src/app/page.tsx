"use client"

import { useDashboardStore } from "@/store/useDashboardStore";
import { useBluetooth } from "@/hooks/useBluetooth";
import { StatCard } from "@/components/dashboard/StatCard";
import { LiveChart } from "@/components/dashboard/LiveChart";
import { BottomNav } from "@/components/layout/BottomNav";
import {
  Thermometer,
  Droplets,
  Wind,
  Activity,
  Volume2,
  Bluetooth,
  BluetoothOff,
  AlertTriangle
} from "lucide-react";

import packageJson from "../../package.json";

export default function Home() {
  const { latestData, history, device, calculatedBPM, stressIndex } = useDashboardStore();
  const { connect, disconnect } = useBluetooth();
  const version = packageJson.version;

  return (
    <main className="min-h-screen bg-base-300 pb-24 lg:pb-0">
      {/* Header */}
      <header className="sticky top-0 z-30 flex h-16 w-full justify-center bg-base-100 bg-opacity-90 backdrop-blur transition-all duration-100 shadow-sm border-b border-base-200">
        <div className="flex w-full px-4 items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 rounded-xl overflow-hidden shadow-lg shadow-primary/20">
              <img src="/icons/icon-192x192.png" alt="Logo" className="w-full h-full object-cover" />
            </div>
            <div className="flex flex-col justify-center">
              <h1 className="text-base sm:text-xl font-black tracking-tight leading-tight">
                스트레스 관리 <span className="text-primary">웨어러블 시스템</span>
              </h1>
              <p className="text-[9px] sm:text-[10px] text-base-content/50 font-bold uppercase tracking-[0.2em] leading-none flex items-center gap-2">
                Smart Dashboard <span className="badge badge-outline badge-xs text-[8px] opacity-70">v{version}</span>
              </p>
            </div>
          </div>

          <button
            onClick={device.connected ? disconnect : connect}
            className={`btn btn-sm ${device.connected ? 'btn-error btn-outline' : 'btn-primary'} gap-2 rounded-full`}
          >
            {device.connected ? <BluetoothOff size={16} /> : <Bluetooth size={16} />}
            {device.connected ? 'Disconnect' : 'Connect Device'}
          </button>
        </div>
      </header>

      <div className="p-4 flex flex-col gap-6 max-w-7xl mx-auto">
        {/* Device Status Banner */}
        {!device.connected && (
          <div className="alert alert-warning shadow-lg rounded-2xl border-none bg-warning/20">
            <AlertTriangle className="text-warning" />
            <span className="text-sm font-semibold">장치에 연결해 주세요. AI 분석을 시작할 준비가 되었습니다.</span>
          </div>
        )}

        {/* Real-time Stat Grid */}
        <section className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
          <StatCard
            title="Temperature"
            value={latestData?.temp ?? "--.-"}
            unit="℃"
            icon={<Thermometer size={20} />}
            colorClass="text-orange-500"
          />
          <StatCard
            title="Breath Rate"
            value={calculatedBPM > 0 ? calculatedBPM : "--"}
            unit="BPM"
            icon={<Activity size={20} />}
            colorClass="text-emerald-500"
          />
          <StatCard
            title="Stress Index"
            value={device.connected ? stressIndex : "--"}
            unit="%"
            icon={<AlertTriangle size={20} />}
            colorClass={stressIndex > 70 ? "text-error" : "text-purple-500"}
          />
          <StatCard
            title="Noise Level"
            value={latestData?.noise ?? "--.-"}
            unit="dB"
            icon={<Volume2 size={20} />}
            colorClass="text-rose-500"
          />
        </section>

        {/* Charts Section */}
        <section className="grid grid-cols-1 lg:grid-cols-2 gap-4">
          <LiveChart
            title="Respiratory Waveform (AI Raw Data)"
            data={history}
            dataKey="resistance"
            color="#10b981"
          />
          <LiveChart
            title="Environment Noise (dB)"
            data={history}
            dataKey="noise"
            color="#f43f5e"
          />
        </section>

        {/* IMU Info Grid */}
        <section className="grid grid-cols-2 sm:grid-cols-3 gap-4">
          <div className="bg-base-100 p-4 rounded-2xl shadow-sm border border-base-200">
            <p className="text-[10px] text-base-content/40 font-bold uppercase mb-1">Impact (G)</p>
            <p className="text-xl font-bold">{latestData ? Math.sqrt(latestData.accelX ** 2 + latestData.accelY ** 2 + latestData.accelZ ** 2).toFixed(2) : "0.00"}</p>
          </div>
          <div className="bg-base-100 p-4 rounded-2xl shadow-sm border border-base-200">
            <p className="text-[10px] text-base-content/40 font-bold uppercase mb-1">Tilt (Pitch)</p>
            <p className="text-xl font-bold">{latestData?.angleX ?? "0.0"}°</p>
          </div>
          <div className="bg-base-100 p-4 rounded-2xl shadow-sm border border-base-200">
            <p className="text-[10px] text-base-content/40 font-bold uppercase mb-1">Roll</p>
            <p className="text-xl font-bold">{latestData?.angleY ?? "0.0"}°</p>
          </div>
        </section>
      </div>

      <BottomNav />
    </main>
  );
}
