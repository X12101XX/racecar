#!/usr/bin/env python3
"""Loopback-only configuration editor. Saves profiles; has no ROS/process/motion endpoint."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import argparse
import errno
import http.client
import io
import json
import secrets
import sys
import urllib.parse
import zipfile

sys.path.insert(0,str(Path(__file__).resolve().parent))
from profile_config import ROOT,default_profile,validate,save_profile,safe_name,MOTION_PRESETS,MOTION_LIMITS


def existing_editor(port):
    """Only reuse an editor that identifies this exact project, without proxying."""
    connection=http.client.HTTPConnection('127.0.0.1',port,timeout=.5)
    try:
        connection.request('GET','/health')
        response=connection.getresponse()
        if response.status!=200:return False
        data=json.loads(response.read(4096))
        return isinstance(data,dict) and data.get('service')=='crossfire-config-editor' and data.get('project')==str(ROOT.resolve())
    except (OSError,ValueError,http.client.HTTPException):return False
    finally:connection.close()


def listen_or_reuse(handler,port=None):
    automatic=port is None
    ports=list(range(8765,8785))+[0] if automatic else [port]
    if automatic:
        # Reuse a fallback instance even if the original default port became free.
        for candidate in ports[:-1]:
            if existing_editor(candidate):return None,candidate
    for candidate in ports:
        try:
            server=ThreadingHTTPServer(('127.0.0.1',candidate),handler)
            return server,server.server_port
        except OSError as error:
            if error.errno!=errno.EADDRINUSE:raise
            if existing_editor(candidate):return None,candidate
            if not automatic:
                raise ValueError(f'端口 {candidate} 已被其他服务或旧版界面占用。可运行 bash tools/configure.sh 自动选端口，或加 --port 0 使用空闲端口。') from None
    raise RuntimeError('无法分配本地端口')


def main():
    ap=argparse.ArgumentParser(description='本机配置界面；默认复用同项目实例或自动选择空闲端口')
    ap.add_argument('--port',type=int,help='指定端口；0表示自动分配空闲端口');a=ap.parse_args()
    if a.port is not None and not 0<=a.port<=65535:ap.error('端口应在0～65535之间')
    token=secrets.token_urlsafe(24)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def respond(self,value,status=200,kind='application/json; charset=utf-8'):
            data=value if isinstance(value,bytes) else json.dumps(value,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Referrer-Policy','no-referrer')
            self.end_headers();self.wfile.write(data)
        def authorized(self):
            host=self.headers.get('Host','').split(':')[0]
            return host in ('127.0.0.1','localhost') and self.headers.get('X-Config-Token')==token
        def do_GET(self):
            parsed=urllib.parse.urlparse(self.path)
            if self.headers.get('Host','').split(':')[0] not in ('localhost','127.0.0.1'):
                return self.respond({'error':'host rejected'},403)
            if parsed.path=='/health':
                return self.respond(dict(service='crossfire-config-editor',project=str(ROOT.resolve())))
            if parsed.path=='/':
                html=(ROOT/'ui/config_editor.html').read_text().replace('__TOKEN__',token)
                return self.respond(html.encode(),kind='text/html; charset=utf-8')
            if not self.authorized():return self.respond({'error':'token required'},403)
            try:
                name=urllib.parse.parse_qs(parsed.query).get('name',['default'])[0]
                if parsed.path=='/api/init':
                    profiles=sorted(p.stem for p in (ROOT/'profiles').glob('*.json'))
                    return self.respond(dict(profile=default_profile(),presets=MOTION_PRESETS,limits=MOTION_LIMITS,profiles=profiles,hardware=json.loads((ROOT/'config/vehicle_profile.json').read_text())))
                safe_name(name);path=ROOT/'profiles'/(name+'.json')
                if parsed.path=='/api/load':return self.respond(json.loads(path.read_text()))
                if parsed.path=='/api/export':
                    p=json.loads(path.read_text());save_profile(name,p)
                    folder=ROOT/'profiles/generated'/name;buf=io.BytesIO()
                    with zipfile.ZipFile(buf,'w',zipfile.ZIP_DEFLATED) as z:
                        for f in sorted(folder.iterdir()):
                            if f.is_file():z.write(f,f.name)
                    return self.respond(buf.getvalue(),kind='application/zip')
                return self.respond({'error':'not found'},404)
            except (ValueError,OSError,KeyError) as e:return self.respond({'error':str(e)},400)
        def do_POST(self):
            if not self.authorized():return self.respond({'error':'token required'},403)
            try:
                length=int(self.headers.get('Content-Length',0))
                if not 0<length<=1000000:raise ValueError('请求体应小于1MB')
                body=json.loads(self.rfile.read(length));profile=body['profile']
                if self.path=='/api/validate':return self.respond(validate(profile))
                if self.path=='/api/save':return self.respond(save_profile(body['name'],profile))
                return self.respond({'error':'not found'},404)
            except (ValueError,OSError,KeyError,TypeError) as e:return self.respond({'error':str(e)},400)
    try:server,port=listen_or_reuse(Handler,a.port)
    except (OSError,ValueError) as error:ap.exit(2,f'配置界面启动失败：{error}\n')
    if server is None:
        print(f'本项目的配置界面已在运行，直接打开：http://127.0.0.1:{port}/\n已复用现有进程。关闭页面不会退出服务；在原启动终端按 Ctrl+C 可退出。',flush=True)
        return
    if a.port is None and port!=8765:
        print('默认端口已被占用，已选择空闲端口；原有进程保持运行。',flush=True)
    print(f'配置界面：http://127.0.0.1:{server.server_port}/\n仅编辑/校验/导出，不连接小车，不启动 ROS。Ctrl+C 退出。',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()

if __name__=='__main__':main()
