export function ErrorBanner({
  message,
  onDismiss,
}: {
  message?: string | null;
  onDismiss?: () => void;
}) {
  if (!message) return null;
  return (
    <div className="p-3 rounded-md border border-red-300 bg-red-50 text-red-700 text-sm">
      <p>{message}</p>
      {onDismiss && (
        <button onClick={onDismiss} className="text-xs text-red-500 underline mt-2">
          Закрыть
        </button>
      )}
    </div>
  );
}
