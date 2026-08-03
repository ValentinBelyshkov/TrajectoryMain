import pathlib

p = pathlib.Path("/opt/main/Trajectory/TWA/client/components/SystemStatus.tsx")
text = p.read_text()

old_block = """            {isLoading ? (
              <Loader2 className="w-3.5 h-3.5 animate-spin" />
            ) : !isOk && canRestart ? (
              <RotateCcw className="w-3.5 h-3.5" />
            ) : (
              label
            )}"""

new_block = """            {isLoading ? (
              <Loader2 className="w-3.5 h-3.5 animate-spin" />
            ) : (
              label
            )}"""

text = text.replace(old_block, new_block)
p.write_text(text)
