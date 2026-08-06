import { useEffect, useState } from "react";
import { getTerraSLAMStatus } from "@/lib/api";

export interface TerraSLAMStatus {
  status: "working" | "warning" | "not_working" | "error";
  publisher_mode: string;
  components: Record<string, string>;
}

export function useTerraSLAMStatus(projectId: string | undefined) {
  const [systemStatus, setSystemStatus] = useState<TerraSLAMStatus | null>(null);

  useEffect(() => {
    if (!projectId) return;

    const fetchStatus = async () => {
      try {
        const status = await getTerraSLAMStatus();
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
