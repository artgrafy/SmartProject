import type { NextConfig } from "next";
const withPWA = require("next-pwa")({
  dest: "public",
  disable: false, // 개발 모드에서도 PWA 작동 확인을 위해 해제
  register: true,
  skipWaiting: true,
});

const nextConfig: NextConfig = {
  /* config options here */
};

export default withPWA(nextConfig);
