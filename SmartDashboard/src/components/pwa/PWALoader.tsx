"use client";

import { useEffect } from "react";

export default function PWALoader() {
    useEffect(() => {
        if (typeof window !== "undefined" && "serviceWorker" in navigator) {
            // 컴포넌트 마운트 시 즉시 서비스 워커 등록 시도 (v2.0.4)
            navigator.serviceWorker
                .register("/sw.js")
                .then((registration) => {
                    console.log("PWA Service Worker registered with scope:", registration.scope);
                })
                .catch((error) => {
                    console.error("PWA Service Worker registration failed:", error);
                });
        }
    }, []);

    return null;
}
