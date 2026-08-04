import {
  Zap,
  Gauge,
  Battery,
  Activity,
  Navigation,
  Signal,
} from "lucide-react";
import { cn } from "@/lib/utils";

interface TelemetryData {
  height: number;
  vx?: number;
  vy?: number;
  vz?: number;
  speed?: number;
  battery: number;
  gpsStatus?: "lock" | "no_fix" | "rtk";
  mode?: "manual" | "auto" | "simulation" | "hover";
  status: "idle" | "recording" | "active" | "error";
}

interface TelemetryBarProps {
  data: TelemetryData;
  className?: string;
}

export function TelemetryBar({ data, className }: TelemetryBarProps) {
  const getStatusColor = (status: string) => {
    switch (status) {
      case "recording":
        return "bg-red-100 text-red-700";
      case "active":
        return "bg-green-100 text-green-700";
      case "error":
        return "bg-red-100 text-red-700";
      default:
        return "bg-gray-100 text-gray-700";
    }
  };

  const getBatteryColor = (battery: number) => {
    if (battery > 60) return "text-green-600";
    if (battery > 30) return "text-yellow-600";
    return "text-red-600";
  };

  const getStatusLabel = (status: string) => {
    switch (status) {
      case "recording":
        return "Запись";
      case "active":
        return "Активен";
      case "error":
        return "Ошибка";
      default:
        return "Ожидание";
    }
  };

  const getGpsIcon = (gpsStatus?: string) => {
    switch (gpsStatus) {
      case "lock":
        return "text-green-500";
      case "rtk":
        return "text-blue-500";
      default:
        return "text-gray-400";
    }
  };

  const speed =
    data.speed ??
    Math.sqrt((data.vx ?? 0) ** 2 + (data.vy ?? 0) ** 2 + (data.vz ?? 0) ** 2);

  return (
    <div
      className={cn(
        "w-full bg-white border-b border-border px-4 py-3 shadow-sm sticky top-0 z-[1100]",
        className,
      )}
    >
      <div className="grid grid-cols-2 md:grid-cols-5 lg:grid-cols-7 gap-3">
        <div className="flex items-center gap-2">
          <div className="p-1.5 bg-blue-100 rounded">
            <Zap className="w-4 h-4 text-blue-600" />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              Высота
            </p>
            <p className="text-sm font-bold text-foreground">
              {data.height.toFixed(1)} м
            </p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <div className="p-1.5 bg-cyan-100 rounded">
            <Gauge className="w-4 h-4 text-cyan-600" />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              Скорость
            </p>
            <p className="text-sm font-bold text-foreground">
              {speed.toFixed(1)} м/с
            </p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <div className="p-1.5 bg-amber-100 rounded">
            <Battery className={cn("w-4 h-4", getBatteryColor(data.battery))} />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              Батарея
            </p>
            <p
              className={cn("text-sm font-bold", getBatteryColor(data.battery))}
            >
              {data.battery}%
            </p>
          </div>
        </div>

        <div className="hidden lg:flex items-center gap-2">
          <div className="p-1.5 bg-green-100 rounded">
            <Signal className={cn("w-4 h-4", getGpsIcon(data.gpsStatus))} />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              GPS
            </p>
            <p className="text-sm font-bold text-foreground">
              {data.gpsStatus === "lock"
                ? "✓"
                : data.gpsStatus === "rtk"
                  ? "RTK"
                  : "—"}
            </p>
          </div>
        </div>

        <div className="hidden lg:flex items-center gap-2">
          <div className="p-1.5 bg-purple-100 rounded">
            <Navigation className="w-4 h-4 text-purple-600" />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              Режим
            </p>
            <p className="text-sm font-bold text-foreground capitalize">
              {data.mode || "manual"}
            </p>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <div className="p-1.5 bg-purple-100 rounded">
            <Activity className="w-4 h-4 text-purple-600" />
          </div>
          <div>
            <p className="text-[10px] text-muted-foreground leading-tight">
              Статус
            </p>
            <p className="text-sm font-bold text-foreground">
              {getStatusLabel(data.status)}
            </p>
          </div>
        </div>

        <div className="flex items-center">
          <div
            className={cn(
              "px-2 py-1 rounded font-semibold text-xs",
              getStatusColor(data.status),
            )}
          >
            {data.status === "recording"
              ? "🔴"
              : data.status === "active"
                ? "✓"
                : ""}{" "}
            {getStatusLabel(data.status)}
          </div>
        </div>
      </div>
    </div>
  );
}
