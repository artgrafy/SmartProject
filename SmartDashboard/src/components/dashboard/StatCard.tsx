import React from 'react';

interface StatCardProps {
    title: string;
    value: string | number;
    unit: string;
    icon: React.ReactNode;
    colorClass?: string;
}

export const StatCard = ({ title, value, unit, icon, colorClass = "text-primary" }: StatCardProps) => {
    return (
        <div className="card bg-base-100 shadow-xl border border-base-200">
            <div className="card-body p-4 flex-row items-center gap-4">
                <div className={`p-3 rounded-2xl bg-base-200 ${colorClass}`}>
                    {icon}
                </div>
                <div>
                    <h2 className="text-xs text-base-content/60 font-medium">{title}</h2>
                    <div className="flex items-baseline gap-1">
                        <span className="text-2xl font-bold tracking-tight">{value}</span>
                        <span className="text-xs text-base-content/40 font-semibold">{unit}</span>
                    </div>
                </div>
            </div>
        </div>
    );
};
