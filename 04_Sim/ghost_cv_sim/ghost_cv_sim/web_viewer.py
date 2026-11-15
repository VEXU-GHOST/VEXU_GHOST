"""Tiny stdlib HTTP server: live MJPEG of the scored frames + running metrics.

Open http://localhost:<port> on the host (the docker-compose file publishes 8090). Lets you
watch the CV results in a normal browser without any GUI forwarding.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = b"""<!doctype html><meta charset=utf-8><title>CV sim</title>
<style>body{font:14px system-ui;margin:0;background:#111;color:#ddd;display:flex;gap:16px;padding:12px;flex-wrap:wrap}
img{max-width:min(900px,100%);border:1px solid #444}table{border-collapse:collapse;margin:6px 0}
td,th{border:1px solid #444;padding:2px 8px;text-align:right}th:first-child,td:first-child{text-align:left}
h3{margin:10px 0 2px}.k{color:#8c8}</style>
<div><img src=/stream.mjpg></div><div id=s style="min-width:340px">loading...</div>
<script>
const f=(v,d=2)=>v==null?'-':(typeof v=='number'?v.toFixed(d):v);
async function tick(){try{const s=await (await fetch('/stats.json')).json();const m=s.summary||{};
let h='<h3>Overall</h3><table><tr><th>frames</th><td>'+(m.frames||0)+'</td></tr>'+
['precision','recall','f1','map50'].map(k=>'<tr><th>'+k+'</th><td class=k>'+f(m[k])+'</td></tr>').join('')+
'<tr><th>TP / FP / FN</th><td>'+[m.tp,m.fp,m.fn].join(' / ')+'</td></tr>'+
'<tr><th>latency ms (p50)</th><td>'+f(m.latency_ms&&m.latency_ms.p50,0)+'</td></tr></table>';
h+='<h3>Per class</h3><table><tr><th></th><th>GT</th><th>P</th><th>R</th><th>AP50</th></tr>'+
Object.entries(m.per_class||{}).map(([c,d])=>'<tr><th>'+c+'</th><td>'+d.gt+'</td><td>'+f(d.precision)+'</td><td>'+f(d.recall)+'</td><td>'+f(d.ap50)+'</td></tr>').join('')+'</table>';
h+='<h3>Recall by distance</h3><table>'+(m.recall_by_distance||[]).map(d=>'<tr><th>'+d.range_m[0]+'-'+(d.range_m[1]==null?'inf':d.range_m[1])+' m</th><td>'+d.gt+'</td><td>'+f(d.recall)+'</td></tr>').join('')+'</table>';
h+='<h3>Frames not scored</h3><table>'+Object.entries(s.drops||{}).map(([k,v])=>'<tr><th>'+k+'</th><td>'+v+'</td></tr>').join('')+'</table>';
document.getElementById('s').innerHTML=h}catch(e){}}
setInterval(tick,1000);tick();
</script>"""


class WebViewer:
    def __init__(self, port):
        self._jpeg = None
        self._seq = 0
        self._stats = b"{}"
        self._cv = threading.Condition()
        viewer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                if self.path == "/stats.json":
                    self._send(viewer._stats, "application/json")
                elif self.path.startswith("/stream.mjpg"):
                    self._stream()
                elif self.path == "/":
                    self._send(PAGE, "text/html")
                else:
                    self.send_error(404)

            def _send(self, body, ctype):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        with viewer._cv:
                            viewer._cv.wait_for(lambda: viewer._seq != seen, timeout=5.0)
                            jpeg, seen = viewer._jpeg, viewer._seq
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                                         % len(jpeg) + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

    def update_frame(self, jpeg_bytes):
        with self._cv:
            self._jpeg = jpeg_bytes
            self._seq += 1
            self._cv.notify_all()

    def update_stats(self, stats):
        self._stats = json.dumps(stats).encode("utf-8")

    def close(self):
        self._server.shutdown()
        self._server.server_close()
