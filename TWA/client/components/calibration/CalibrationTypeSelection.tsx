import { MapPin, MousePointerClick } from "lucide-react";

interface CalibrationTypeSelectionProps {
  onSelect: (type: "manual" | "auto") => void;
  onBack: () => void;
}

export function CalibrationTypeSelection({ onSelect, onBack }: CalibrationTypeSelectionProps) {
  return (
    <div className="flex flex-col h-full">
      <div className="bg-gradient-to-r from-blue-600 to-blue-800 text-white p-6 shrink-0">
        <h2 className="text-2xl font-bold mb-2">Выбор типа калибровки</h2>
        <p className="text-blue-100">
          Выберите метод калибровки в зависимости от высоты полёта
        </p>
      </div>

      <div className="flex-1 p-8 overflow-auto">
        <div className="max-w-2xl mx-auto space-y-6">
          <button
            onClick={() => {}}
            disabled
            className="w-full p-8 bg-white border-2 border-slate-200 rounded-2xl opacity-60 cursor-not-allowed text-left group"
          >
            <div className="flex items-start gap-5">
              <div className="w-16 h-16 bg-slate-100 rounded-xl flex items-center justify-center shrink-0">
                <MapPin className="w-8 h-8 text-slate-400" />
              </div>
              <div className="flex-1">
                <div className="flex items-center gap-3 mb-2">
                  <h3 className="text-xl font-bold text-foreground">Автоматическая калибровка</h3>
                  <span className="px-3 py-1 bg-slate-100 text-slate-500 text-xs font-semibold rounded-full">
                    🚧 В разработке
                  </span>
                </div>
                <p className="text-muted-foreground mb-4">
                  Для полётов на высоте от 100 метров. Система автоматически загрузит карту местности 
                  и сопоставит её с данными дрона.
                </p>
                <div className="flex flex-wrap gap-2 text-sm">
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-500">
                    🗺️ Загрузка карты по региону
                  </span>
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-500">
                    🔍 Автоматическое сопоставление
                  </span>
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-500">
                    ✈️ Подходит для аэрофотосъёмки
                  </span>
                </div>
              </div>
            </div>
          </button>

          <button
            onClick={() => onSelect("manual")}
            className="w-full p-8 bg-white border-2 border-slate-200 rounded-2xl hover:border-slate-400 hover:bg-slate-50 transition-all text-left group"
          >
            <div className="flex items-start gap-5">
              <div className="w-16 h-16 bg-slate-100 rounded-xl flex items-center justify-center shrink-0 group-hover:bg-slate-200 transition-colors">
                <MousePointerClick className="w-8 h-8 text-slate-600" />
              </div>
              <div className="flex-1">
                <div className="flex items-center gap-3 mb-2">
                  <h3 className="text-xl font-bold text-foreground">Ручная калибровка</h3>
                  <span className="px-3 py-1 bg-amber-100 text-amber-700 text-xs font-semibold rounded-full">
                    любые высоты
                  </span>
                </div>
                <p className="text-muted-foreground mb-4">
                  Классический метод калибровки с ручным выбором контрольных точек на изображении 
                  и на карте. Требуется запись видео и тестовый запуск SLAM.
                </p>
                <div className="flex flex-wrap gap-2 text-sm">
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-600">
                    📹 Запись видео
                  </span>
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-600">
                    👆 Ручной выбор точек
                  </span>
                  <span className="px-2 py-1 bg-slate-100 rounded text-slate-600">
                    🎯 Высокая точность
                  </span>
                </div>
              </div>
            </div>
          </button>
        </div>
      </div>

      <div className="p-6 border-t bg-slate-50 shrink-0">
        <button
          onClick={onBack}
          className="w-full max-w-md mx-auto text-muted-foreground hover:text-foreground py-2 transition-colors"
        >
          ← Вернуться
        </button>
      </div>
    </div>
  );
}
