import { useState } from "react";
import { useQuery, useMutation, useQueryClient } from "@tanstack/react-query";
import { Link, useNavigate } from "react-router-dom";
import { Plus, Trash2, Eye, Settings } from "lucide-react";
import { ProjectModal, ProjectType } from "@/components/ProjectModal";
import { SettingsModal } from "@/components/SettingsModal";
import { Button } from "@/components/ui/button";
import { SystemStatus } from "@/components/SystemStatus";
import {
  getProjects,
  createProject,
  deleteProject,
  uploadProjectVideo,
  type Project,
} from "@/lib/api";
import { toast } from "sonner";

export default function Index() {
  const queryClient = useQueryClient();
  const navigate = useNavigate();
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [uploadProgress, setUploadProgress] = useState<{
    progress: number;
    remainingTime: number;
  } | null>(null);

  const { data: projects = [], isLoading } = useQuery({
    queryKey: ["projects"],
    queryFn: getProjects,
  });

  const createMutation = useMutation({
    mutationFn: async ({
      name,
      type,
      videoFile,
    }: {
      name: string;
      type: ProjectType;
      videoFile?: File;
    }): Promise<Project> => {
      let project = await createProject(name, type);
      if (videoFile) {
        // Connect to telemetry websocket for progress updates
        const wsUrl = `${import.meta.env.VITE_WS_URL || "ws://localhost:9000"}/api/telemetry/ws/${project.id}`;
        const ws = new WebSocket(wsUrl);
        
        // Wait for WebSocket to open to ensure we don't miss early progress messages
        await new Promise((resolve) => {
          const timeout = setTimeout(resolve, 1000); // Fallback after 1s
          ws.onopen = () => {
            clearTimeout(timeout);
            console.log("✅ Telemetry WebSocket for progress connected");
            resolve(true);
          };
        });

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            if (data.project_id === project.id && data.type === "ffmpeg_progress") {
              setUploadProgress({
                progress: data.progress,
                remainingTime: data.remaining_time,
              });
            }
          } catch (e) {
            console.error("Failed to parse progress message:", e);
          }
        };

        try {
          const uploadResult = await uploadProjectVideo(project.id, videoFile);
          project = { ...project, videoFilename: uploadResult.filename };
        } finally {
          ws.close();
          setUploadProgress(null);
        }
      }
      return project;
    },
    onSuccess: (newProject) => {
      queryClient.setQueryData(["projects"], (old: Project[] = []) => [
        newProject,
        ...old,
      ]);
      queryClient.invalidateQueries({ queryKey: ["projects"] });
      toast.success("Проект успешно создан", { duration: 1000 });
      setIsModalOpen(false);
      navigate(`/project/${newProject.id}`);
    },
    onError: (error: any) => {
      toast.error(`Ошибка при создании проекта: ${error.message}`, {
        duration: 1000,
      });
    },
  });

  const deleteMutation = useMutation({
    mutationFn: deleteProject,
    onSuccess: (_, deletedId) => {
      queryClient.setQueryData(["projects"], (old: Project[] = []) =>
        old.filter((p) => p.id !== deletedId),
      );
      queryClient.invalidateQueries({ queryKey: ["projects"] });
      toast.success("Проект удален", { duration: 1000 });
    },
    onError: (error: any) => {
      toast.error(`Ошибка при удалении проекта: ${error.message}`, {
        duration: 1000,
      });
    },
  });

  const handleCreateProject = async (
    name: string,
    type: ProjectType,
    videoFile?: File
  ) => {
    await createMutation.mutateAsync({ name, type, videoFile });
  };

  const handleDeleteProject = (id: string) => {
    if (confirm("Вы уверены, что хотите удалить этот проект?")) {
      deleteMutation.mutate(id);
    }
  };

  const formatDate = (date: Date) => {
    return new Intl.DateTimeFormat("ru-RU", {
      year: "numeric",
      month: "short",
      day: "numeric",
      hour: "2-digit",
      minute: "2-digit",
    }).format(date);
  };

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-50 to-blue-50">
      {/* Header */}
      <header className="bg-white border-b border-border shadow-sm sticky top-0 z-50">
        <div className="max-w-7xl mx-auto px-6 py-5 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="w-10 h-10 bg-gradient-to-br from-primary to-secondary rounded-lg flex items-center justify-center">
              <span className="text-white font-bold text-xl">🎯</span>
            </div>
            <div>
              <h1 className="text-2xl font-bold text-foreground">Одометрия</h1>
              <p className="text-sm text-muted-foreground">
                Визуальная навигация дрона
              </p>
            </div>
          </div>
          <div className="flex items-center gap-3">
            <SystemStatus />
            <Button
              variant="outline"
              onClick={() => setIsSettingsOpen(true)}
              className="gap-2"
            >
              <Settings className="w-4 h-4" />
              Настройки
            </Button>
            <Button
              onClick={() => setIsModalOpen(true)}
              className="btn-primary gap-2"
            >
              <Plus className="w-4 h-4" />
              Новый проект
            </Button>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-6 py-12">
        {isLoading ? (
          <div className="text-center py-20">
            <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
            <p className="text-muted-foreground">Загрузка проектов...</p>
          </div>
        ) : projects.length === 0 ? (
          <div className="text-center py-20">
            <div className="w-16 h-16 bg-blue-100 rounded-full flex items-center justify-center mx-auto mb-4">
              <span className="text-3xl">📦</span>
            </div>
            <h2 className="text-2xl font-bold text-foreground mb-2">
              Нет проектов
            </h2>
            <p className="text-muted-foreground mb-6 max-w-sm mx-auto">
              Создайте первый проект, чтобы начать работу с визуальной
              одометрией
            </p>
            <Button
              onClick={() => setIsModalOpen(true)}
              className="btn-primary gap-2"
            >
              <Plus className="w-4 h-4" />
              Создать первый проект
            </Button>
          </div>
        ) : (
          <div>
            <div className="mb-8">
              <h2 className="text-xl font-bold text-foreground mb-2">
                Мои проекты
              </h2>
              <p className="text-muted-foreground">
                {projects.length} {
                  projects.length % 10 === 1 && projects.length % 100 !== 11
                    ? "проект"
                    : "проектов"
                }
              </p>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
              {projects.map((project) => (
                <div
                  key={project.id}
                  className="bg-white rounded-xl border border-border p-6 card-hover"
                >
                  <div className="flex items-start justify-between mb-4">
                    <div className="flex items-center gap-3 flex-1">
                      <div className="w-12 h-12 bg-gradient-to-br from-primary/10 to-secondary/10 rounded-lg flex items-center justify-center">
                        <span className="text-xl">
                          {project.type === "камера" ? "📷" : "🎬"}
                        </span>
                      </div>
                      <div className="flex-1 min-w-0">
                        <h3 className="font-bold text-foreground truncate text-lg">
                          {project.name}
                        </h3>
                        <p className="text-sm text-muted-foreground">
                          {project.type === "камера" ? "Камера" : "Симуляция"}
                        </p>
                      </div>
                    </div>
                  </div>

                  <div className="mb-4 p-3 bg-blue-50 rounded-lg">
                    <p className="text-xs text-muted-foreground">
                      Создано: {formatDate(project.createdAt)}
                    </p>
                  </div>

                  <div className="flex gap-2">
                    <Link
                      to={`/project/${project.id}`}
                      className="flex-1 inline-flex items-center justify-center gap-2 px-4 py-2 bg-primary text-white rounded-lg font-semibold hover:bg-primary/90 transition-colors"
                    >
                      <Eye className="w-4 h-4" />
                      Открыть
                    </Link>
                    <button
                      onClick={() => handleDeleteProject(project.id)}
                      className="inline-flex items-center justify-center gap-2 px-4 py-2 text-red-600 hover:bg-red-50 rounded-lg font-semibold transition-colors border border-red-200"
                    >
                      <Trash2 className="w-4 h-4" />
                    </button>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}
      </main>

      <ProjectModal
        open={isModalOpen}
        onOpenChange={setIsModalOpen}
        onCreateProject={handleCreateProject}
        isLoading={createMutation.isPending}
        progress={uploadProgress?.progress}
        remainingTime={uploadProgress?.remainingTime}
      />
      
      <SettingsModal
        open={isSettingsOpen}
        onOpenChange={setIsSettingsOpen}
      />
    </div>
  );
}
