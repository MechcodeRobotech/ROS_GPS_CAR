import ast, json, math, sys, types
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'gps_localize_ws/src/gps_localize'))
from gps_localize import geo
class Msg:
    def __init__(self, data=None):
        self.data=data; self.linear=types.SimpleNamespace(x=0.,y=0.,z=0.); self.angular=types.SimpleNamespace(x=0.,y=0.,z=0.)
class Pub:
    def __init__(self): self.messages=[]
    def publish(self,m): self.messages.append(m)
def load(file,cls):
    tree=ast.parse((ROOT/file).read_text())
    keep=[x for x in tree.body if isinstance(x,(ast.ClassDef,ast.FunctionDef,ast.Assign))]
    ns=dict(Node=object, json=json, math=math, geo=geo, Twist=Msg,String=Msg,Int32=Msg, Bool=Msg, tel=types.SimpleNamespace(HEADING_REF_NAMES={}))
    exec(compile(ast.Module(body=keep,type_ignores=[]),str(file),'exec'),ns)
    return ns[cls]
Nav=load('gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py','WaypointNav')
def nav():
    n=Nav.__new__(Nav); n.now=lambda:100.; n.log=lambda *a:None; n.pub_cmd=Pub(); n._stop_sent=False
    n.state='ALIGN'; n.index=0; n.waypoints=[{'name':'A','lat':10.,'lon':10.},{'name':'B','lat':10.001,'lon':10.001}]
    n.temp_target=None; n.align_target=0.; n.align_start=99.; n.align_why='reached'; n.heading_deg=0.; n.heading_tolerance=15.; n.estop=False
    n.last_distance=0.; n.loop_waypoints=False; n.align_timeout=20.; n.inside_since=None
    return n
n=nav(); n.tick_align(); print('ALIGN advance:',n.state,'index=',n.index,'align_target=',n.align_target)
try:n.tick()
except Exception as e:print('next tick:',type(e).__name__,str(e))
n=nav(); n.require_fix=False; n.fix=None; n.state='RUNNING'
try:n.tick()
except Exception as e:print('require_fix=false, no position:',type(e).__name__,str(e))
n=nav(); n.require_fix=True; n.allow_dead_reckoning=True; n.dead_reckon={'lat':10.,'lon':10.}; n.fix=(10.,10.); n.fix_time=1.; n.fix_timeout=3.
print('99-second-old dead reckoning usable:',n.fix_usable())
n=nav(); n.state='DISCOVER'; n.discover_start=99.; n.discover_from=(10.,10.); n.fix=(10.,10.); n.heading_ref=0; n.discover_max_s=12.; n.discover_distance=2.; n.discover_speed=.15
n.fix_usable=lambda:(False,'no usable GPS'); n.tick_discover(); print('DISCOVER with unusable GPS: commanded speed=',n.pub_cmd.messages[-1].linear.x)
n=nav()
try:n.on_cmd(Msg(json.dumps({'action':'goto','index':'bad'})))
except Exception as e:print('bad nav index:',type(e).__name__,str(e))
Web=load('gps_localize_ws/src/gps_localize/gps_localize/web_server.py','WebServerNode')
w=Web.__new__(Web); w.motion_blocker=lambda:None; w.manual_speed=.18; w.manual_turn=45.; w.pub_manual=Pub()
print('manual NaN:',w.api_manual({'forward':'NaN','turn':0},{}))
Odom=load('gps_localize_ws/src/gps_localize/gps_localize/gps_odom.py','GpsOdom')
o=Odom.__new__(Odom); o.datum=(10.,10.); o.fused_en=(0.,0.); o.pub_datum=Pub(); o.get_logger=lambda:types.SimpleNamespace(info=lambda *a:None)
o.set_datum(11.,11.); print('datum shifted 1 degree: unchanged fused EN=',o.fused_en,'new interpreted latlon=',geo.enu_to_latlon(*o.fused_en,*o.datum))
from gps_localize.configstore import ConfigStore
Web.api_config_set.__globals__['GROUPS']=('nav',)
import tempfile
with tempfile.TemporaryDirectory() as td:
    store=ConfigStore(str(ROOT/'gps_localize_ws/src/gps_localize/config'),td)
    w.store=store; w.allow_config_edit=True; w.pub_config_updated=Pub(); w.add_log=lambda *a:None
    print('malformed nav config HTTP status=',w.api_config_set({'group':'nav','data':{'nav':{'cruise_speed_mps':'bad'}}},{})[0])
    n=nav(); n.store=store
    try:n._load_config()
    except Exception as e:print('nav reload:',type(e).__name__,str(e))
