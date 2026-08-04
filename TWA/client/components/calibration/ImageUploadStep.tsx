import { Upload } from "lucide-react";

interface ImageUploadStepProps {
  onUpload: (file: File) => void;
  uploadError: string | null;
  isUploading: boolean;
  onErrorDismiss: () => void;
  onFileSelect: (inputRef: HTMLInputElement) => void;
}

export function ImageUploadStep({
  onUpload,
  uploadError,
  isUploading,
  onErrorDismiss,
  onFileSelect,
}: ImageUploadStepProps) {
  return (
    <div className="p-12 max-w-lg mx-auto w-full">
      <div className="text-center mb-8">
        <div className="w-20 h-20 bg-purple-100 rounded-full flex items-center justify-center mx-auto mb-4">
          <span className="text-4xl">📷</span>
        </div>
        <h2 className="text-2xl font-bold text-foreground mb-2">
          Загрузка изображения
        </h2>
        <p className="text-muted-foreground">
          Выберите изображение с дрона для калибровки
        </p>
      </div>

        {uploadError && (
          <div className="mb-6 p-4 bg-red-50 border border-red-200 rounded-lg">
            <p className="text-sm text-red-600">{uploadError}</p>
            <button
              onClick={onErrorDismiss}
              className="text-xs text-red-500 underline mt-2"
            >
              Закрыть
            </button>
          </div>
        )}

        <div className="space-y-6">
          <div
            onClick={() => {
              const input = document.createElement("input");
              input.type = "file";
              input.accept = "image/jpeg,image/png,image/jpg";
              input.onchange = (e) => {
                const file = (e.target as HTMLInputElement).files?.[0];
                if (file) onUpload(file);
              };
              input.click();
            }}
            className="border-2 border-dashed border-border rounded-xl p-12 text-center cursor-pointer hover:border-primary hover:bg-blue-50/50 transition-all"
          >
            {isUploading ? (
              <div className="flex flex-col items-center">
                <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mb-4"></div>
                <p className="text-muted-foreground">Загрузка...</p>
              </div>
            ) : (
              <>
                <Upload className="w-12 h-12 text-muted-foreground mx-auto mb-4" />
                <p className="font-semibold text-foreground mb-2">
                  Нажмите для выбора файла
                </p>
                <p className="text-sm text-muted-foreground">
                  Поддерживаются форматы: JPG, PNG
                </p>
              </>
            )}
          </div>
        </div>
    </div>
  );
}
