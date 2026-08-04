import { ArrowLeft, Settings } from "lucide-react";
import type { Project } from "@/lib/api";
import { SystemStatus } from "./SystemStatus";

interface ProjectHeaderProps {
  project: Project;
  onBack: () => void;
  onSettingsClick?: () => void;
}

export function ProjectHeader({ project, onBack, onSettingsClick }: ProjectHeaderProps) {
  return (
    <header className="bg-white border-b border-border px-6 py-4 flex items-center justify-between">
      <div className="flex items-center gap-4">
        <button
          onClick={onBack}
          className="p-2 hover:bg-muted rounded-lg transition-colors"
          title="Назад"
        >
          <ArrowLeft className="w-6 h-6 text-foreground" />
        </button>
        <div>
          <h1 className="text-xl font-bold text-foreground">{project.name}</h1>
          <p className="text-sm text-muted-foreground">
            {project.type === "камера" ? "Камера" : "Симуляция"}
            {project.calibrationStatus === "calibrated" && (
              <span className="ml-2 text-green-600">✓ Откалибровано</span>
            )}
          </p>
        </div>
      </div>
      <div className="flex items-center gap-3">
        <SystemStatus />
        <button 
          onClick={onSettingsClick}
          className="p-2 hover:bg-muted rounded-lg transition-colors"
        >
          <Settings className="w-6 h-6 text-foreground" />
        </button>
      </div>
    </header>
  );
}
