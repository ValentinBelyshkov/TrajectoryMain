import { useState, useEffect } from "react";
import {
  AlertDialog,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogHeader,
  AlertDialogTitle,
} from "@/components/ui/alert-dialog";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from "@/components/ui/select";

import { Progress } from "@/components/ui/progress";

export type ProjectType = "камера" | "симуляция";

interface ProjectModalProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  onCreateProject: (name: string, type: ProjectType, videoFile?: File) => void;
  isLoading?: boolean;
  progress?: number;
  remainingTime?: number;
}

export function ProjectModal({
  open,
  onOpenChange,
  onCreateProject,
  isLoading,
  progress,
  remainingTime,
}: ProjectModalProps) {
  const [projectName, setProjectName] = useState("");
  const [projectType, setProjectType] = useState<ProjectType | "">("");
  const [videoFile, setVideoFile] = useState<File | null>(null);
  const [step, setStep] = useState<"type" | "video">("type");

  useEffect(() => {
    if (!open) {
      setProjectName("");
      setProjectType("");
      setVideoFile(null);
      setStep("type");
    }
  }, [open]);

  const handleCreate = () => {
    if (!projectName || !projectType) return;

    if (projectType === "симуляция" && !videoFile) {
      alert("Пожалуйста, выберите видеофайл");
      return;
    }

    onCreateProject(
      projectName,
      projectType as ProjectType,
      videoFile || undefined,
    );
  };

  const handleReset = () => {
    setProjectName("");
    setProjectType("");
    setVideoFile(null);
    setStep("type");
    onOpenChange(false);
  };

  const handleNext = () => {
    if (!projectName || !projectType) return;
    if (projectType === "симуляция") {
      setStep("video");
    } else {
      handleCreate();
    }
  };

  return (
    <AlertDialog open={open} onOpenChange={onOpenChange}>
      <AlertDialogContent className="max-w-md">
        <AlertDialogHeader>
          <AlertDialogTitle>Создать новый проект</AlertDialogTitle>
          <AlertDialogDescription>
            {step === "type"
              ? "Введите название проекта и выберите тип"
              : "Выберите видеофайл для симуляции"}
          </AlertDialogDescription>
        </AlertDialogHeader>

        {isLoading ? (
          <div className="py-8 px-2 text-center">
            <div className="mb-6 relative">
              <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-primary mx-auto"></div>
              {progress !== undefined && (
                <div className="absolute inset-0 flex items-center justify-center text-xs font-bold">
                  {Math.round(progress * 100)}%
                </div>
              )}
            </div>
            
            <h3 className="font-bold text-lg text-foreground mb-2">
              Обработка видео...
            </h3>
            
            {progress !== undefined && (
              <div className="space-y-3 mb-4">
                <Progress value={progress * 100} className="h-2" />
                <div className="flex justify-between text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">
                  <span>Прогресс: {Math.round(progress * 100)}%</span>
                  {remainingTime !== undefined && remainingTime > 0 && (
                    <span>Осталось: ~{Math.round(remainingTime)} сек.</span>
                  )}
                </div>
              </div>
            )}
            
            <p className="text-sm text-muted-foreground">
              Пожалуйста, подождите, идет нарезка кадров для калибровки
            </p>
          </div>
        ) : step === "type" ? (
          <div className="space-y-4 py-4">
            <div className="space-y-2">
              <Label htmlFor="project-name">Название проекта</Label>
              <Input
                id="project-name"
                placeholder="Например: Проект-1"
                value={projectName}
                onChange={(e) => setProjectName(e.target.value)}
                className="border-border"
              />
            </div>

            <div className="space-y-2">
              <Label htmlFor="project-type">Тип проекта</Label>
              <Select
                value={projectType}
                onValueChange={(value) =>
                  setProjectType(value as ProjectType | "")
                }
              >
                <SelectTrigger id="project-type">
                  <SelectValue placeholder="Выберите тип" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="камера">📷 Камера</SelectItem>
                  <SelectItem value="симуляция">🎬 Симуляция</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </div>
        ) : (
          <div className="space-y-4 py-4">
            <div className="space-y-2">
              <Label htmlFor="video-file">Видеофайл</Label>
              <Input
                id="video-file"
                type="file"
                accept="video/*"
                onChange={(e) => setVideoFile(e.target.files?.[0] || null)}
                className="border-border"
              />
              {videoFile && (
                <p className="text-sm text-muted-foreground">
                  Выбран: {videoFile.name}
                </p>
              )}
            </div>
          </div>
        )}

        <div className="flex gap-2 justify-end">
          <AlertDialogCancel onClick={handleReset} disabled={isLoading}>Отмена</AlertDialogCancel>
          {!isLoading && (
            step === "type" ? (
              <Button
                onClick={handleNext}
                disabled={!projectName || !projectType}
              >
                {projectType === "симуляция" ? "Далее" : "Создать"}
              </Button>
            ) : (
              <>
                <Button variant="outline" onClick={() => setStep("type")}>
                  Назад
                </Button>
                <Button onClick={handleCreate} disabled={!videoFile}>
                  Создать
                </Button>
              </>
            )
          )}
        </div>
      </AlertDialogContent>
    </AlertDialog>
  );
}
