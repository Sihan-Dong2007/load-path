// Dev only: stands in for `@footron/controls-client` so the real phone UI in
// controls/lib can be bundled and driven locally. useMessaging() gives the
// same { sendMessage } the real hook does, but over a same-origin BroadcastChannel
// that dev/fake-messaging.js listens on in the wall page. Like the real hook it
// also hands every message coming back from the wall to the callback it is given.
import { useEffect, useRef } from "react";

const channel = typeof BroadcastChannel === "function" ? new BroadcastChannel("loadpath-dev") : null;
export function useMessaging(onMessage) {
  const latest = useRef(onMessage);
  latest.current = onMessage;
  useEffect(() => {
    if (!channel) return undefined;
    const listener = (event) => latest.current && latest.current(event.data);
    channel.addEventListener("message", listener);
    return () => channel.removeEventListener("message", listener);
  }, []);
  return { sendMessage: (body) => channel && channel.postMessage(body) };
}
