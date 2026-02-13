import React from 'react';
import { LayoutDashboard, History, Settings, Bell } from 'lucide-react';

export const BottomNav = () => {
    return (
        <div className="btm-nav btm-nav-md bg-base-200 border-t border-base-300 lg:hidden">
            <button className="active text-primary">
                <LayoutDashboard size={20} />
                <span className="btm-nav-label text-xs">Home</span>
            </button>
            <button>
                <Bell size={20} />
                <span className="btm-nav-label text-xs">Alerts</span>
            </button>
            <button>
                <History size={20} />
                <span className="btm-nav-label text-xs">History</span>
            </button>
            <button>
                <Settings size={20} />
                <span className="btm-nav-label text-xs">Settings</span>
            </button>
        </div>
    );
};
