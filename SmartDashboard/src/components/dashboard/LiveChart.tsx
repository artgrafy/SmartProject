"use client"

import React from 'react';
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis, Tooltip, CartesianGrid } from 'recharts';
import { SensorData } from '@/types/sensor';

interface LiveChartProps {
    data: SensorData[];
    dataKey: keyof SensorData;
    title: string;
    color: string;
}

export const LiveChart = ({ data, dataKey, title, color }: LiveChartProps) => {
    // Recharts expects a plain object
    const chartData = [...data].reverse().map(d => ({
        time: new Date(d.timestamp).toLocaleTimeString([], { hour12: false, minute: '2-digit', second: '2-digit' }),
        value: d[dataKey]
    }));

    return (
        <div className="card bg-base-100 shadow-xl border border-base-200 overflow-hidden">
            <div className="card-body p-4 gap-0">
                <h3 className="text-sm font-bold text-base-content/70 mb-4">{title}</h3>
                <div className="h-48 w-full">
                    <ResponsiveContainer width="100%" height="100%">
                        <AreaChart data={chartData}>
                            <defs>
                                <linearGradient id={`color${String(dataKey)}`} x1="0" y1="0" x2="0" y2="1">
                                    <stop offset="5%" stopColor={color} stopOpacity={0.3} />
                                    <stop offset="95%" stopColor={color} stopOpacity={0} />
                                </linearGradient>
                            </defs>
                            <CartesianGrid strokeDasharray="3 3" vertical={false} stroke="rgba(255,255,255,0.05)" />
                            <XAxis
                                dataKey="time"
                                hide
                            />
                            <YAxis
                                domain={['auto', 'auto']}
                                hide
                            />
                            <Tooltip
                                contentStyle={{ backgroundColor: '#1f2937', border: 'none', borderRadius: '8px', fontSize: '12px' }}
                                itemStyle={{ color: '#fff' }}
                            />
                            <Area
                                type="monotone"
                                dataKey="value"
                                stroke={color}
                                fillOpacity={1}
                                fill={`url(#color${String(dataKey)})`}
                                strokeWidth={3}
                                isAnimationActive={false}
                            />
                        </AreaChart>
                    </ResponsiveContainer>
                </div>
            </div>
        </div>
    );
};
