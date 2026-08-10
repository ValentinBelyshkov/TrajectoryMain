import { useQuery } from "@tanstack/react-query";
import { getTerraSLAMStatus } from "@/lib/api";
import { cn } from "@/lib/utils";

const COMPONENT_LABELS: Record<string, string> = {
  slam: "SLM",
  relay: "RLY",
  publisher_folder: "FLD",
  publisher_realsense: "RLS",
  gps_bridge: "GPS",
  rosbridge: "ROS",
};

interface SystemStatusProps {
  /**
   * Project the status belongs to. Determines which publisher circle is shown:
   * simulation projects use the folder publisher (FLD), camera projects use
   * the RealSense publisher (RLS).
   */
  projectId?: string;
}

export function SystemStatus({ projectId }: SystemStatusProps = {}) {
  const { data, isError } = useQuery({
    queryKey: ["system-status", projectId ?? null],
    queryFn: () => getTerraSLAMStatus(projectId),
    refetchInterval: 1000,
  });

  if (isError || !data) {
    return null;
  }

  const { components, publisher_mode } = data;

  // Define specific order for components
  const componentOrder = [
    "slam",
    "relay",
    publisher_mode === "folder" ? "publisher_folder" : "publisher_realsense",
    "gps_bridge",
    "rosbridge",
  ];

  const visibleComponents = componentOrder
    .filter(name => components[name] !== undefined)
    .map(name => [name, components[name]]);

  return (
    <div className="flex items-center gap-2">
      {visibleComponents.map(([name, status]) => {
        const upper = status.toUpperCase();
        // "RUNNING (pid ...)" -> green, "ERROR ..." -> red,
        // everything else (incl. "NOT RUNNING (idle)") -> amber.
        // NOTE: use startsWith for RUNNING so "NOT RUNNING" is not matched.
        const colorClass = upper.startsWith("RUNNING")
          ? "bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.5)]"
          : upper.includes("ERROR")
            ? "bg-red-500 shadow-[0_0_8px_rgba(239,68,68,0.5)]"
            : "bg-amber-500 shadow-[0_0_8px_rgba(245,158,11,0.5)]";
        const label = COMPONENT_LABELS[name] || name.substring(0, 3).toUpperCase();

        return (
          <div
            key={name}
            title={`${name}: ${status}`}
            className={cn(
              "w-8 h-8 rounded-full flex items-center justify-center text-[10px] font-bold text-white transition-colors",
              colorClass
            )}
          >
            {label}
          </div>
        );
      })}
    </div>
  );
}
