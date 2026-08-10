import { useEffect, useState } from "react";
import { getTerraSLAMStatus } from "@/lib/api";

export interface TerraSLAMStatus {
  status: "working" | "warning" | "not_working" | "error";
  publisher_mode: string;
  components: Record<string, string>;
}

/**
 * Whether a given component is actually running on the backend.
 * The gateway reports runtime state strings like "RUNNING (pid 360030)"
 * or "NOT RUNNING (idle)", so a plain "RUNNING" substring check would
 * also match "NOT RUNNING". Exclude the negative form explicitly.
 */
export function isComponentRunning(
  status: TerraSLAMStatus | null | undefined,
  component: string,
): boolean {
  const state = status?.components?.[component];
  if (!state) return false;
  const upper = state.toUpperCase();
  return upper.includes("RUNNING") && !upper.includes("NOT RUNNING");
}

/** Whether the SLAM component itself is currently running. */
export function isSlamRunning(status: TerraSLAMStatus | null | undefined): boolean {
  return isComponentRunning(status, "slam");
}

export function useTerraSLAMStatus(projectId: string | undefined) {
  const [systemStatus, setSystemStatus] = useState<TerraSLAMStatus | null>(null);

  useEffect(() => {
    if (!projectId) return;

    const fetchStatus = async () => {
      try {
        const status = await getTerraSLAMStatus(projectId);
        setSystemStatus({
          status: status.system_status as TerraSLAMStatus["status"],
          publisher_mode: status.publisher_mode,
          components: status.components,
        });
      } catch (err) {
        console.error("Failed to fetch TerraSLAM status:", err);
        setSystemStatus({
          status: "error",
          publisher_mode: "unknown",
          components: {},
        });
      }
    };

    fetchStatus();
    const interval = setInterval(fetchStatus, 3000);
    return () => clearInterval(interval);
  }, [projectId]);

  return systemStatus;
}
