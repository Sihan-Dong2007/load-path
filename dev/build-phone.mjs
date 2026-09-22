// Bundles the real phone UI (footron/controls/lib/index.js) for a local smoke test,
// with the Footron messaging hook swapped for dev/controls-client-stub.js.
//   cd dev && npm install && npm run build-phone
// then serve with `python3 serve.py 8950 --dev` and open /dev/phone-react.html.
import { build } from "esbuild";
import { fileURLToPath } from "node:url";
import path from "node:path";

const here = path.dirname(fileURLToPath(import.meta.url));
await build({
  entryPoints: [path.join(here, "phone-entry.js")],
  bundle: true,
  outfile: path.join(here, "build", "phone.js"),
  loader: { ".js": "jsx" },
  jsx: "automatic",
  jsxImportSource: "@emotion/react",
  // The phone UI lives in ../footron, outside this folder, so tell esbuild where the dependencies are.
  nodePaths: [path.join(here, "node_modules")],
  alias: { "@footron/controls-client": path.join(here, "controls-client-stub.js") },
  define: { "process.env.NODE_ENV": '"development"' },
  logLevel: "info",
});
