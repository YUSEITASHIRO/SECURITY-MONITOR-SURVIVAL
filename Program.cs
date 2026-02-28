// ============================================================================
// HORROR GAME  Security Monitor Survival
// Raylib-cs 7.0.2 / .NET 8
// ============================================================================
using System;
using System.Collections.Generic;
using System.Numerics;
using Raylib_cs;
using static Raylib_cs.Raylib;

const int   SW = 1280, SH = 720, MTW = 640, MTH = 480;
const float Tile = 4.0f, HallH = 4.0f;
const float TurnDur = 30f, GracePer = 3f;
const int   MaxLives = 3, MaxTurns = 5;
const float MonSpd   = 1.2f;
const float CamFov   = 80f;
const float ArrowSpd = 60f;     // camera rotation speed deg/s
static Color C(byte r,byte g,byte b,byte a)=>new Color(r,g,b,a);

InitWindow(SW, SH, "HORROR GAME  Security Monitor Survival");
SetTargetFPS(60);

// ─── State ───────────────────────────────────────────────────────────────────
GameState gs = GameState.Menu;
int turn=1, lives=MaxLives, activeCh=0;
float turnTimer=TurnDur, graceTimer=0f;
bool monInView=false;
Random rng=new();

List<Vector3> path=new();
// Per-segment info for building
List<(Vector3 start, Vector3 dir, int len)> segs=new();
// Cameras: position+yaw (yaw controlled by arrow keys)
List<Vector3> camPos=new();
List<float>   camYaw=new();   // degrees, 0=+Z, 90=+X
List<string>  camName=new();
List<bool>    camBroken=new();

// Monster
float monT=0f, monSpeed=MonSpd, spdTimer=0f;
Vector3 monPos=Vector3.Zero;
float monFaceYaw=0f;   // direction monster faces (degrees)
bool monTurnedBack=false; float monTurnTimer=0f;

// Scare state
int scarePhase=0, livesAtScare=0, curScareType=0;
float scareTimer=0f;
bool scareBlack=false; float blackTimer=0f;
const float BlackDur=1.4f;
float playerYaw=0f, playerPitch=0f;
Camera3D scareCam=new(){Position=Vector3.Zero,Target=Vector3.UnitZ,Up=new(0,1,0),FovY=72f,Projection=CameraProjection.Perspective};
float scareInitYaw=0f;
// Scare3: chase
float chasePlayerT=0f, chaseMonsterT=0f;
float chaseSpeed=4.5f, chaseMonSpeed=5.2f;
bool  chaseRunning=false; // player must press Space to run
int   chaseIntroPhase=0;  // 0=looking back, 1=turned forward, 2=running

RenderTexture2D monRT=LoadRenderTexture(MTW,MTH);

// ─── Build Camera3D from pos+yaw ─────────────────────────────────────────────
Camera3D MakeCam3D(Vector3 pos, float yawDeg)
{
    float yr=yawDeg*MathF.PI/180f;
    Vector3 fwd=new(MathF.Sin(yr),0,-MathF.Cos(yr));
    // Camera is on ceiling: look down the corridor at an angle
    // Target is ahead along corridor and below camera height
    Vector3 target=pos+fwd*Tile*4f+new Vector3(0,-HallH*0.7f,0);
    return new Camera3D{Position=pos,Target=target,Up=new(0,1,0),FovY=CamFov,Projection=CameraProjection.Perspective};
}

// ─── Generate hallway ─────────────────────────────────────────────────────────
// Points (intersection boxes) list
List<Vector3> points = new();

void GenerateHallway()
{
    path.Clear(); segs.Clear(); camPos.Clear(); camYaw.Clear(); camName.Clear(); camBroken.Clear(); points.Clear();

    // ── Parameters scaling with turn (Max exactly 10) ──
    int maxPoints    = Math.Min(10, 4 + turn * 2);        // turn1=6, turn3=10, max=10
    int maxDirsBase  = Math.Min(1 + turn, 4);             // turn1=2, turn3=4, max=4
    int minCorLen    = 3;
    int maxCorLen    = 5 + turn * 2;                       // turn1=7, turn3=11, turn5=15
    int numCams      = Math.Min(10, Math.Min(3 + turn * 2, maxPoints)); // Max=10

    // ── Directions ──
    Vector3[] allDirs = { new(1,0,0), new(-1,0,0), new(0,0,1), new(0,0,-1) };

    // ── Step 1: Place point A at origin ──
    points.Add(Vector3.Zero);
    Queue<int> queue = new();
    queue.Enqueue(0);

    // Helper: check if a direction from a point already has a segment
    bool HasSegInDir(Vector3 from, Vector3 dir2)
    {
        foreach (var (ss, sd, sl) in segs)
        {
            Vector3 se = ss + sd * ((sl - 1) * Tile);
            // Segment starts at 'from' going in 'dir2' or opposite
            if ((ss - from).Length() < Tile * 0.3f && Vector3.Dot(sd, dir2) > 0.9f) return true;
            if ((se - from).Length() < Tile * 0.3f && Vector3.Dot(-sd, dir2) > 0.9f) return true;
        }
        return false;
    }

    // ── Step 2: BFS expand from each point ──
    while (queue.Count > 0 && points.Count < maxPoints)
    {
        int pi = queue.Dequeue();
        Vector3 pPos = points[pi];

        int numDirs = 1 + rng.Next(maxDirsBase);
        numDirs = Math.Min(numDirs, 4);

        // Shuffle directions
        int[] dOrder = { 0, 1, 2, 3 };
        for (int i = 3; i > 0; i--) { int j = rng.Next(i + 1); int t2 = dOrder[i]; dOrder[i] = dOrder[j]; dOrder[j] = t2; }

        for (int di = 0; di < numDirs && points.Count < maxPoints; di++)
        {
            Vector3 dir = allDirs[dOrder[di]];

            // Skip if already have a segment in this direction from this point
            if (HasSegInDir(pPos, dir)) continue;

            int len = minCorLen + rng.Next(maxCorLen - minCorLen + 1);
            Vector3 endPt = pPos + dir * (len * Tile);

            // Check if endpoint is near an existing point ON THE SAME AXIS
            int existingPt = -1;
            for (int ep = 0; ep < points.Count; ep++)
            {
                if (ep == pi) continue;
                Vector3 ep3 = points[ep];
                // Must be on same axis line
                bool axisOk = false;
                if (MathF.Abs(dir.X) > 0.5f) // moving along X: Z must match
                    axisOk = MathF.Abs(ep3.Z - pPos.Z) < Tile * 0.3f && MathF.Sign(ep3.X - pPos.X) == MathF.Sign(dir.X);
                else // moving along Z: X must match
                    axisOk = MathF.Abs(ep3.X - pPos.X) < Tile * 0.3f && MathF.Sign(ep3.Z - pPos.Z) == MathF.Sign(dir.Z);

                if (axisOk)
                {
                    float axisDist = MathF.Abs(dir.X) > 0.5f ? MathF.Abs(ep3.X - pPos.X) : MathF.Abs(ep3.Z - pPos.Z);
                    if (axisDist >= Tile * 2f && axisDist <= len * Tile + Tile)
                    {
                        existingPt = ep;
                        len = (int)MathF.Round(axisDist / Tile);
                        break;
                    }
                }
            }

            if (existingPt >= 0)
            {
                // Connect to existing point on same axis
                if (len < 2) continue;
            }
            else
            {
                // Place new point at end
                endPt = pPos + dir * (len * Tile);

                // Check not too close to existing points
                bool tooClose = false;
                foreach (var ep2 in points)
                    if ((ep2 - endPt).Length() < Tile * 1.5f && (ep2 - endPt).Length() > Tile * 0.1f)
                    { tooClose = true; break; }
                if (tooClose)
                {
                    len = Math.Max(2, len - 2);
                    endPt = pPos + dir * (len * Tile);
                }
                points.Add(endPt);
                queue.Enqueue(points.Count - 1);
            }

            // Create segment: goes from pPos to pPos+dir*len*Tile, covering len+1 tiles
            segs.Add((pPos, dir, len + 1));
        }
    }

    // ── Step 3: Place box markers at each point (visual only, for 3D) ──
    // (Points are rendered as small cubes in DrawHallway)

    // ── Step 4: Assign cameras to random points ──
    List<int> ptIndices = new();
    for (int i = 0; i < points.Count; i++) ptIndices.Add(i);
    // Shuffle
    for (int i = ptIndices.Count - 1; i > 0; i--)
    { int j = rng.Next(i + 1); int t3 = ptIndices[i]; ptIndices[i] = ptIndices[j]; ptIndices[j] = t3; }

    for (int ci = 0; ci < numCams && ci < ptIndices.Count; ci++)
    {
        Vector3 pp = points[ptIndices[ci]];
        Vector3 camP = pp + new Vector3(0, HallH * 0.85f, 0);
        // Initial yaw: toward first connected corridor direction FROM this point
        float yaw = 0f;
        bool foundDir = false;
        foreach (var (ss, sd, sl) in segs)
        {
            Vector3 segEnd = ss + sd * ((sl - 1) * Tile);
            if ((ss - pp).Length() < Tile * 0.5f)
            {
                // Segment starts here: look along segment direction
                yaw = MathF.Atan2(sd.X, -sd.Z) * 180f / MathF.PI;
                foundDir = true; break;
            }
            else if ((segEnd - pp).Length() < Tile * 0.5f)
            {
                // Segment ends here: look back along segment (opposite direction)
                yaw = MathF.Atan2(-sd.X, sd.Z) * 180f / MathF.PI;
                foundDir = true; break;
            }
        }
        camPos.Add(camP); camYaw.Add(yaw);
        camName.Add($"CH{ci}");
        
        // Sometimes a camera will break from turn 2
        bool broken = false;
        if(turn >= 2 && rng.NextDouble() < 0.2f) broken = true; 
        camBroken.Add(broken);
    }

    // ── Build path by walking through segment graph (always on corridors) ──
    {
        path.Clear();
        List<Vector3> walk = new();
        List<List<(int to, int seg)>> adj = new();
        for(int i=0; i<points.Count; i++) adj.Add(new());
        
        for(int si=0; si<segs.Count; si++)
        {
            var (s, dir, len) = segs[si];
            Vector3 e = s + dir * ((len - 1) * Tile);
            int p1 = points.FindIndex(p => (p - s).Length() < 1f);
            int p2 = points.FindIndex(p => (p - e).Length() < 1f);
            if (p1 >= 0 && p2 >= 0)
            {
                adj[p1].Add((p2, si));
                adj[p2].Add((p1, si));
            }
        }

        HashSet<int> visitedEdges = new();
        void DFS(int curr)
        {
            foreach(var edge in adj[curr])
            {
                if (visitedEdges.Contains(edge.seg)) continue;
                visitedEdges.Add(edge.seg);
                
                var (ss, sd, sl) = segs[edge.seg];
                bool fromStart = (points[curr] - ss).Length() < 1f;
                
                // Walk from curr to next
                for (int t = 1; t < sl; t++)
                {
                    int tt = fromStart ? t : (sl - 1 - t);
                    walk.Add(ss + sd * (tt * Tile));
                }
                
                DFS(edge.to);
                
                // Walk back
                for (int t = sl - 2; t >= 0; t--)
                {
                    int tt = fromStart ? t : (sl - 1 - t);
                    walk.Add(ss + sd * (tt * Tile));
                }
            }
        }
        
        if (points.Count > 0)
        {
            walk.Add(points[0]);
            DFS(0);
        }
        path.AddRange(walk);
    }

    // Monster start at point A
    monT = 0f; monSpeed = MonSpd; spdTimer = 0f;
    monPos = path.Count > 0 ? path[0] + new Vector3(0, 0.5f, 0) : Vector3.Zero;
    monFaceYaw = 0f; monTurnedBack = false; monTurnTimer = 0f;

    // Guarantee: ensure monster visible by at least one camera
    bool anyVis = false;
    for (int ci = 0; ci < camPos.Count; ci++)
        if (IsMonsterVisible(ci)) { anyVis = true; break; }
    if (!anyVis && camPos.Count > 0)
    {
        float bestD = float.MaxValue; int bestC = 0;
        for (int ci = 0; ci < camPos.Count; ci++)
        {
            float d = (camPos[ci] - monPos).Length();
            if (d < bestD) { bestD = d; bestC = ci; }
        }
        Vector3 toM = monPos - camPos[bestC];
        camYaw[bestC] = MathF.Atan2(toM.X, -toM.Z) * 180f / MathF.PI;
    }
    activeCh = 0;
}

// ─── Monster update ───────────────────────────────────────────────────────────
void UpdateMonster(float dt)
{
    if(path.Count<2)return;
    spdTimer-=dt;
    if(spdTimer<=0f)
    {
        float rv=(float)rng.NextDouble();
        
        // Intelligent monster behavior
        if(monInView && turn >= 2)
        {
            // Find the nearest corner (direction change)
            int fwdIdx = Math.Min((int)monT + 1, path.Count - 1);
            for(int i = (int)monT; i < path.Count - 2; i++) {
                Vector3 d1 = Vector3.Normalize(path[i+1] - path[i]);
                Vector3 d2 = Vector3.Normalize(path[i+2] - path[i+1]);
                if((d1 - d2).LengthSquared() > 0.1f) { fwdIdx = i + 1; break; }
            }
            
            int bwdIdx = Math.Max((int)monT - 1, 0);
            for(int i = (int)monT; i > 1; i--) {
                Vector3 d1 = Vector3.Normalize(path[i] - path[i-1]);
                Vector3 d2 = Vector3.Normalize(path[i-1] - path[i-2]);
                if((d1 - d2).LengthSquared() > 0.1f) { bwdIdx = i - 1; break; }
            }
            
            float distFwd = fwdIdx - monT;  if (distFwd < 1f) distFwd = 1f;
            float distBwd = monT - bwdIdx;  if (distBwd < 1f) distBwd = 1f;
            
            // Keep moving in direction if we already have momentum, else pick closest
            float curSign = (MathF.Abs(monSpeed) > 0.1f) ? MathF.Sign(monSpeed) : (distFwd <= distBwd ? 1f : -1f);
            
            // Dash past the corner significantly to completely break LOS
            if (curSign > 0) {
                monSpeed = MonSpd * 5.5f; 
                spdTimer = (distFwd / monSpeed) + 0.8f; 
            } else {
                monSpeed = -MonSpd * 5.5f; 
                spdTimer = (distBwd / MathF.Abs(monSpeed)) + 0.8f;
            }
        }
        else if (activeCh >=0 && activeCh < camBroken.Count && camBroken[activeCh] && turn >= 2)
        {
            monSpeed=MonSpd*2.8f;spdTimer=1.0f; // move fast confidently when watching broken cam
        }
        else
        {
            if(rv<0.02f){monSpeed=0f;spdTimer=0.3f+(float)rng.NextDouble()*0.5f;}
            else if(rv<0.08f){monSpeed=-MonSpd*1.5f;spdTimer=0.5f+(float)rng.NextDouble()*1.5f;} 
            else if(rv<0.15f){monSpeed=-MonSpd*0.5f;spdTimer=0.5f+(float)rng.NextDouble()*1f;}   
            else if(rv<0.30f){monSpeed=MonSpd*0.6f;spdTimer=1f+(float)rng.NextDouble()*1.5f;}     
            else if(rv<0.60f){monSpeed=MonSpd*1.2f;spdTimer=1f+(float)rng.NextDouble()*2f;}       
            else if(rv<0.85f){monSpeed=MonSpd*2.5f;spdTimer=0.8f+(float)rng.NextDouble()*1f;}     
            else {monSpeed=MonSpd*4.0f;spdTimer=0.3f+(float)rng.NextDouble()*0.5f;}               
        }
    }
    monT+=monSpeed*dt;
    monT=Math.Clamp(monT,0f,path.Count-1.01f);
    if(monT<=0.1f&&monSpeed<0f)monSpeed=MathF.Abs(monSpeed);
    if(monT>=path.Count-1.1f&&monSpeed>0f)monSpeed=-monSpeed*0.5f;

    int idx=(int)monT; float frac=monT-idx;
    int nx=Math.Min(idx+1,path.Count-1);
    monPos=Vector3.Lerp(path[idx],path[nx],frac)+new Vector3(0,0.5f,0);

    // Face direction of travel
    if(idx<path.Count-1)
    {
        Vector3 travelDir=path[nx]-path[idx];
        if(travelDir.Length()>0.01f)
        {
            float tYaw=MathF.Atan2(travelDir.X,-travelDir.Z)*180f/MathF.PI;
            if(monSpeed<0) tYaw+=180f; // reverse = face other way
            monFaceYaw=tYaw;
        }
    }

    // Horror: if camera is behind monster, random chance to stop & turn around
    monTurnTimer-=dt;
    if(!monTurnedBack && monTurnTimer<=0f && monInView)
    {
        // Check if camera is behind monster
        if(activeCh>=0&&activeCh<camPos.Count)
        {
            Vector3 toCamera=camPos[activeCh]-monPos;
            float monFacingRad=monFaceYaw*MathF.PI/180f;
            Vector3 monFwd=new(MathF.Sin(monFacingRad),0,-MathF.Cos(monFacingRad));
            float dot=Vector3.Dot(Vector3.Normalize(new(toCamera.X,0,toCamera.Z)),monFwd);
            if(dot<-0.3f && rng.NextDouble()<0.015f) // camera is behind
            {
                monTurnedBack=true; monTurnTimer=2.5f+(float)rng.NextDouble()*2f;
                monSpeed=0f; spdTimer=monTurnTimer;
                monFaceYaw+=180f; // snap face toward camera
            }
        }
    }
    if(monTurnedBack&&monTurnTimer<=0f)
    {
        monTurnedBack=false; monFaceYaw+=180f; // turn back to travel dir
    }
}

// ─── Visibility: FOV direction + straight line through connected segments ────
// Find ALL segments a point belongs to (it can be on multiple at intersections)
List<int> FindAllSegments(Vector3 p)
{
    List<int> result = new();
    for(int si=0;si<segs.Count;si++)
    {
        var(start,dir,len)=segs[si];
        Vector3 end=start+dir*((len-1)*Tile);
        float sMinX=MathF.Min(start.X,end.X)-Tile*0.6f;
        float sMaxX=MathF.Max(start.X,end.X)+Tile*0.6f;
        float sMinZ=MathF.Min(start.Z,end.Z)-Tile*0.6f;
        float sMaxZ=MathF.Max(start.Z,end.Z)+Tile*0.6f;
        if(p.X>=sMinX&&p.X<=sMaxX&&p.Z>=sMinZ&&p.Z<=sMaxZ)result.Add(si);
    }
    return result;
}

bool IsMonsterVisible(int ch)
{
    if(ch<0||ch>=camPos.Count)return false;
    if(camBroken[ch]) return false; // broken camera sees nothing
    Vector3 cp=camPos[ch];
    Vector3 cpF=new(cp.X,0,cp.Z); // camera floor pos
    Vector3 mF=new(monPos.X,0,monPos.Z); // monster floor pos
    float dist=(mF-cpF).Length();

    // Very close = always visible
    if(dist<Tile*1.5f)return true;

    // Monster must be on same axis line as camera (same X or same Z)
    float dx=MathF.Abs(mF.X-cpF.X), dz=MathF.Abs(mF.Z-cpF.Z);
    bool sameXaxis = dz < Tile * 0.6f; // same Z row, separated along X
    bool sameZaxis = dx < Tile * 0.6f; // same X column, separated along Z
    if(!sameXaxis && !sameZaxis)return false;

    // Direction from camera to monster
    Vector3 toMon=Vector3.Normalize(mF-cpF);

    // FOV check: must be within camera's view cone
    float camRad=camYaw[ch]*MathF.PI/180f;
    Vector3 camFwd=new(MathF.Sin(camRad),0,-MathF.Cos(camRad));
    float dot=Vector3.Dot(toMon,camFwd);
    if(dot<MathF.Cos(CamFov*0.5f*MathF.PI/180f))return false;

    // Corridor continuity check: walk from camera to monster in Tile steps
    // Every step must be on at least one segment
    int steps=(int)(dist/Tile)+1;
    for(int s=0;s<=steps;s++)
    {
        float t=MathF.Min(s*Tile,dist);
        Vector3 checkPt=cpF+toMon*t;
        bool onSeg=false;
        for(int si=0;si<segs.Count;si++)
        {
            var(start,dir,len)=segs[si];
            Vector3 end=start+dir*((len-1)*Tile);
            float mnX=MathF.Min(start.X,end.X)-Tile*0.6f;
            float mxX=MathF.Max(start.X,end.X)+Tile*0.6f;
            float mnZ=MathF.Min(start.Z,end.Z)-Tile*0.6f;
            float mxZ=MathF.Max(start.Z,end.Z)+Tile*0.6f;
            if(checkPt.X>=mnX&&checkPt.X<=mxX&&checkPt.Z>=mnZ&&checkPt.Z<=mxZ){onSeg=true;break;}
        }
        if(!onSeg)return false; // gap in corridor = can't see through
    }
    return true;
}

// ─── Draw hallway 3D ──────────────────────────────────────────────────────────
void DrawHallway()
{
    Color fc=C(140,135,125,255),wc=C(165,158,145,255),cc=C(120,115,108,255);
    // Draw each segment as floor+ceiling+walls (only between the points)
    foreach(var(start,dir,len) in segs)
    {
        float segmentLen = (len - 2) * Tile; // Distance strictly between the intersection boxes
        if(segmentLen <= 0) continue; 
        
        Vector3 end = start + dir * ((len - 1) * Tile);
        Vector3 mid = (start + end) * 0.5f;
        Vector3 perp = new(-dir.Z, 0, dir.X);
        
        if(MathF.Abs(dir.X) > 0.5f)
        {
            DrawCube(mid, segmentLen, 0.15f, Tile, fc); // floor
            DrawCube(mid + new Vector3(0, HallH, 0), segmentLen, 0.15f, Tile, cc); // ceiling
        }
        else
        {
            DrawCube(mid, Tile, 0.15f, segmentLen, fc); // floor
            DrawCube(mid + new Vector3(0, HallH, 0), Tile, 0.15f, segmentLen, cc); // ceiling
        }
        
        Vector3 wMid = mid + new Vector3(0, HallH * 0.5f, 0);
        Vector3 ws = MathF.Abs(dir.X) > 0.5f ? new(segmentLen, HallH, 0.15f) : new(0.15f, HallH, segmentLen);
        DrawCubeV(wMid + perp * (Tile * 0.5f), ws, wc); // right wall
        DrawCubeV(wMid - perp * (Tile * 0.5f), ws, wc); // left wall
    }
    // Intersection boxes at each point (same width as corridors)
    foreach(var pp in points)
    {
        DrawCube(pp+new Vector3(0,0.02f,0),Tile,0.15f,Tile,C(100,95,85,255)); // floor
        DrawCube(pp+new Vector3(0,HallH,0),Tile,0.15f,Tile,C(90,85,78,255)); // ceiling
        // Draw walls on sides that DON'T connect to a corridor
        bool hasXp=false,hasXn=false,hasZp=false,hasZn=false;
        foreach(var(ss,sd,sl) in segs)
        {
            Vector3 se=ss+sd*((sl-1)*Tile);
            if((ss-pp).Length()<Tile*0.5f||(se-pp).Length()<Tile*0.5f)
            {
                Vector3 segDir=(ss-pp).Length()<Tile*0.5f?sd:-sd;
                if(segDir.X>0.5f)hasXp=true; if(segDir.X<-0.5f)hasXn=true;
                if(segDir.Z>0.5f)hasZp=true; if(segDir.Z<-0.5f)hasZn=true;
            }
        }
        Vector3 wMid=pp+new Vector3(0,HallH*0.5f,0);
        if(!hasXp)DrawCube(wMid+new Vector3(Tile*0.5f,0,0),0.15f,HallH,Tile,C(165,158,145,255));
        if(!hasXn)DrawCube(wMid-new Vector3(Tile*0.5f,0,0),0.15f,HallH,Tile,C(165,158,145,255));
        if(!hasZp)DrawCube(wMid+new Vector3(0,0,Tile*0.5f),Tile,HallH,0.15f,C(165,158,145,255));
        if(!hasZn)DrawCube(wMid-new Vector3(0,0,Tile*0.5f),Tile,HallH,0.15f,C(165,158,145,255));
    }
    // Camera mounts
    for(int ci=0;ci<camPos.Count;ci++)
    {
        DrawCube(camPos[ci],0.4f,0.25f,0.4f,C(20,20,30,255));
        DrawSphere(camPos[ci]-new Vector3(0,0.18f,0),0.08f,C(220,30,30,255));
    }
}

// ─── Draw monster ─────────────────────────────────────────────────────────────
void DrawMonster(Vector3 pos, float faceYaw)
{
    Color body=C(8,5,10,255);
    DrawCylinder(pos,0.35f,0.28f,1.8f,8,body);
    DrawSphere(pos+new Vector3(0,1.9f,0),0.38f,body);
    // Eyes face the monster's facing direction
    float yr=faceYaw*MathF.PI/180f;
    Vector3 ef=new(MathF.Sin(yr),0,-MathF.Cos(yr));
    Vector3 eb=pos+new Vector3(0,1.8f,0);
    Vector3 er=new(-ef.Z,0,ef.X);
    DrawSphere(eb+ef*0.3f+er*0.14f,0.09f,C(255,20,20,255));
    DrawSphere(eb+ef*0.3f-er*0.14f,0.09f,C(255,20,20,255));
}

// ─── Draw monitor 3D view ─────────────────────────────────────────────────────
void RenderMonitor(int ch)
{
    if(ch<0||ch>=camPos.Count)return;
    BeginTextureMode(monRT);
    ClearBackground(C(15,14,12,255));
    if (camBroken[ch])
    {
        // White noise for broken camera
        for(int y=0; y<MTH; y+=8) {
            for (int x=0; x<MTW; x+=8) {
                byte bc = (byte)rng.Next(20, 220);
                DrawRectangle(x,y,8,8,C(bc,bc,bc,255));
            }
        }
        DrawText("CAMERA SIGNAL LOST", MTW/2-100, MTH/2, 20, Color.Red);
    }
    else
    {
        Camera3D cam3=MakeCam3D(camPos[ch],camYaw[ch]);
        BeginMode3D(cam3);
        DrawHallway();
        DrawMonster(monPos,monFaceYaw);
        EndMode3D();
    }
    
    // CCTV overlays
    for(int y=0;y<MTH;y+=4)DrawRectangle(0,y,MTW,2,C(0,0,0,28));
    DrawRectangle(0,0,MTW,MTH,C(0,15,4,22));
    float t2=(float)GetTime();
    for(int ni=0;ni<200;ni++){
        int nx2=(int)((MathF.Sin(t2*19f+ni*0.29f)*0.5f+0.5f)*MTW);
        int ny2=(int)((MathF.Cos(t2*13f+ni*0.43f)*0.5f+0.5f)*MTH);
        DrawPixel(nx2,ny2,C((byte)rng.Next(35,100),(byte)rng.Next(35,100),(byte)rng.Next(35,100),50));
    }
    DrawRectangleGradientH(0,0,(int)(MTW*0.18f),MTH,C(0,0,0,140),C(0,0,0,0));
    DrawRectangleGradientH((int)(MTW*0.82f),0,(int)(MTW*0.18f),MTH,C(0,0,0,0),C(0,0,0,140));
    DrawRectangleGradientV(0,0,MTW,(int)(MTH*0.15f),C(0,0,0,140),C(0,0,0,0));
    DrawRectangleGradientV(0,(int)(MTH*0.85f),MTW,(int)(MTH*0.15f),C(0,0,0,0),C(0,0,0,140));
    // Corner crosses
    void Cross(int cx,int cy){DrawLineEx(new(cx-13,cy),new(cx+13,cy),1.5f,C(170,170,170,90));DrawLineEx(new(cx,cy-13),new(cx,cy+13),1.5f,C(170,170,170,90));}
    Cross(18,18);Cross(MTW-18,18);Cross(18,MTH-18);Cross(MTW-18,MTH-18);
    DrawText(camName[ch],10,10,20,C(220,220,220,200));
    string ts=$"{DateTime.Now:yyyy-MM-dd  HH:mm:ss}";
    int tw=MeasureText(ts,14);
    DrawText(ts,MTW-tw-8,10,14,C(200,200,200,160));
    DrawText(ts,(MTW-tw)/2,MTH-24,14,C(200,200,200,150));
    if((int)(GetTime()*2)%2==0){DrawCircle(MTW-20,MTH-14,6,C(220,30,30,255));DrawText("REC",MTW-58,MTH-20,14,C(220,30,30,255));}
    DrawText($"FOV {(int)CamFov}deg",10,MTH-22,12,C(140,200,110,160));
    EndTextureMode();
}

// ─── Draw minimap with FOV cones ──────────────────────────────────────────────
void DrawMinimapWithFOV(int rpX,int rpY,int mmW,int mmH)
{
    if(segs.Count<1)return;
    // Compute bounds from ALL segment endpoints (not path)
    float minX=float.MaxValue,maxX=float.MinValue,minZ=float.MaxValue,maxZ=float.MinValue;
    foreach(var(start,dir,len) in segs)
    {
        Vector3 end=start+dir*((len-1)*Tile);
        if(start.X<minX)minX=start.X;if(start.X>maxX)maxX=start.X;
        if(start.Z<minZ)minZ=start.Z;if(start.Z>maxZ)maxZ=start.Z;
        if(end.X<minX)minX=end.X;if(end.X>maxX)maxX=end.X;
        if(end.Z<minZ)minZ=end.Z;if(end.Z>maxZ)maxZ=end.Z;
    }
    // Also include monster
    if(monPos.X<minX)minX=monPos.X;if(monPos.X>maxX)maxX=monPos.X;
    if(monPos.Z<minZ)minZ=monPos.Z;if(monPos.Z>maxZ)maxZ=monPos.Z;
    // Add margin
    minX-=Tile;maxX+=Tile;minZ-=Tile;maxZ+=Tile;
    float rX=MathF.Max(maxX-minX,1f),rZ=MathF.Max(maxZ-minZ,1f);
    float sc=MathF.Min(mmW/rX,mmH/rZ)*0.85f;
    float ox=rpX+(mmW-rX*sc)*0.5f, oz=rpY+(mmH-rZ*sc)*0.5f;
    Vector2 M(Vector3 v)=>new(ox+(v.X-minX)*sc,oz+(v.Z-minZ)*sc);

    // Path (segment-based)
    foreach(var(start,dir,len) in segs)
    {
        Vector3 end=start+dir*((len-1)*Tile);
        DrawLineEx(M(start),M(end),4f,C(50,90,50,200));
    }
    // Intersection points
    foreach(var pp in points)
    {
        Vector2 pm=M(pp);
        DrawRectangle((int)(pm.X-4),(int)(pm.Y-4),8,8,C(90,130,90,200));
        DrawRectangleLinesEx(new Rectangle(pm.X-4,pm.Y-4,8,8),1,C(60,100,60,255));
    }

    // FOV cones for all cameras (far-reaching)
    for(int ci=0;ci<camPos.Count;ci++)
    {
        Vector2 cp2=M(camPos[ci]);
        float ca=camYaw[ci]*MathF.PI/180f;
        float hf=CamFov*0.5f*MathF.PI/180f;
        float vd=MathF.Max(mmW,mmH)*0.8f; // draw big cone to edge of minimap
        bool sel=ci==activeCh;
        Color fc2=sel?C(255,255,100,40):C(100,200,100,18);
        for(int f=0;f<12;f++){
            float a0=ca-hf+f*(CamFov*MathF.PI/180f/12f);
            float a1=ca-hf+(f+1)*(CamFov*MathF.PI/180f/12f);
            Vector2 p0=cp2+new Vector2(MathF.Sin(a0),-MathF.Cos(a0))*vd;
            Vector2 p1=cp2+new Vector2(MathF.Sin(a1),-MathF.Cos(a1))*vd;
            DrawTriangle(cp2,p0,p1,fc2);
        }
        // FOV edge lines
        Vector2 fL=cp2+new Vector2(MathF.Sin(ca-hf),-MathF.Cos(ca-hf))*vd;
        Vector2 fR=cp2+new Vector2(MathF.Sin(ca+hf),-MathF.Cos(ca+hf))*vd;
        DrawLineEx(cp2,fL,1.2f,sel?C(255,230,80,120):C(100,200,100,50));
        DrawLineEx(cp2,fR,1.2f,sel?C(255,230,80,120):C(100,200,100,50));
        Color cc2 = sel ? C(255,220,60,240) : C(125,195,125,175);
        if (camBroken[ci]) cc2 = sel ? C(255,100,100,240) : C(200,50,50,175); // Broken camera UI
        
        DrawCircleV(cp2,sel?5f:3.5f,cc2);
        DrawText($"{ci}",(int)cp2.X+5,(int)cp2.Y-6,9,cc2);
    }
    // Monster dot
    Vector2 mm=M(monPos);
    DrawCircleV(mm,6f,C(255,40,40,235));
    DrawCircleLinesV(mm,6f,Color.Red);
}

// ─── HUD ──────────────────────────────────────────────────────────────────────
void DrawHUD()
{
    DrawRectangle(0,0,SW,50,C(0,0,0,195));
    DrawRectangle(0,50,SW,1,C(45,75,45,210));
    DrawText($"TURN {turn}/{MaxTurns}",14,13,22,Color.White);
    for(int i=0;i<MaxLives;i++)
        DrawText("*",SW-28-i*22,12,26,i<lives?C(230,50,50,255):C(65,30,30,255));
    if(gs==GameState.Playing){
        float pct=turnTimer/TurnDur; int bx=SW/2-180,by=15;
        DrawRectangle(bx,by,360,20,C(20,20,20,220));
        Color bc=pct>0.5f?C(48,200,78,255):pct>0.25f?C(228,178,28,255):C(218,48,48,255);
        DrawRectangle(bx+2,by+2,(int)(356*pct),16,bc);
        DrawText($"{turnTimer:F1}s",bx+366,by,18,Color.White);
    }
    if(gs==GameState.Playing&&!monInView){
        float dp=graceTimer/GracePer;
        byte wA=(byte)(155+(int)(MathF.Sin((float)GetTime()*8f)*90));
        Color wc=dp>0.6f?C(255,28,28,wA):C(255,198,28,wA);
        string wt=$"!! MONSTER OUT OF VIEW  {GracePer-graceTimer:F1}s !!";
        int ww=MeasureText(wt,22);
        DrawRectangle((SW-ww)/2-10,54,ww+20,32,C(0,0,0,160));
        DrawText(wt,(SW-ww)/2,60,22,wc);
    }
    int btnY=SH-54; DrawRectangle(0,btnY-4,SW,58,C(0,0,0,185));
    int btnW=58,bsX2=(SW-camPos.Count*(btnW+5))/2;
    for(int i=0;i<camPos.Count&&i<10;i++){
        int bx2=bsX2+i*(btnW+5); bool sel=i==activeCh;
        DrawRectangle(bx2,btnY,btnW,44,sel?C(38,155,58,230):C(20,20,20,220));
        DrawRectangleLinesEx(new Rectangle(bx2,btnY,btnW,44),sel?2:1,sel?C(75,215,95,255):C(60,60,60,200));
        
        Color tc = C(150,150,150,255);
        if (camBroken[i]) tc = C(255,100,100,255);
        else if (sel) tc = Color.White;
        
        DrawText(camBroken[i]?"[x]":$"[{i}]",bx2+6,btnY+3,13,camBroken[i]?C(255,80,80,255):C(135,135,135,255));
        DrawText(camBroken[i]?"ERR":camName[i],bx2+5,btnY+19,13,tc);
    }
    DrawText("[0]-[9] Switch Cam  |  Arrow Keys: Rotate Camera",12,SH-18,13,C(100,100,100,255));
}

// ─── Helpers ──────────────────────────────────────────────────────────────────
void StartTurn(){GenerateHallway(); activeCh=0; turnTimer=TurnDur;graceTimer=0f;gs=GameState.Ready;}

void UpdateScareCam()
{
    float yr=(scareInitYaw+playerYaw)*MathF.PI/180f;
    float pr=playerPitch*MathF.PI/180f;
    scareCam.Target=scareCam.Position+new Vector3(MathF.Sin(yr)*MathF.Cos(pr),MathF.Sin(pr),-MathF.Cos(yr)*MathF.Cos(pr));
}

void TriggerScare()
{
    livesAtScare=lives; lives--;
    gs=GameState.Scare; scareTimer=0f; scarePhase=0; scareBlack=false; blackTimer=0f;
    playerYaw=0f; playerPitch=0f;

    // Scare camera: placed in a separate "room" area far from corridor
    Vector3 scareRoomBase=new Vector3(500,0,500); // far away from hallway
    curScareType = rng.Next(6); // Randomly select from 6 types

    if(livesAtScare > 1) // ─ Normal Scares (1 to 6) ─
    {
        scareCam.Position=scareRoomBase+new Vector3(curScareType * 30f, 1.6f, 0); 
        scareInitYaw=0f; // facing +Z initially
        UpdateScareCam();
    }
    else // ─ Scare 3: corridor chase ─
    {
        chasePlayerT=0f; chaseMonsterT=-30f; // monster starts FAR behind
        chaseSpeed=4.5f; chaseMonSpeed=5.2f;
        chaseRunning=false; chaseIntroPhase=0;
        scareCam.Position=new Vector3(600,1.6f,0);
        scareInitYaw=0f;   // Base forward is 0
        playerYaw=180f;    // Start by looking BEHIND at the monster
        playerPitch=0f;
        UpdateScareCam();
    }
}

// ─── Bootstrap ────────────────────────────────────────────────────────────────
StartTurn(); gs=GameState.Menu;

// ─── Main loop ────────────────────────────────────────────────────────────────
while(!WindowShouldClose())
{
    float dt=GetFrameTime();

    switch(gs)
    {
        case GameState.Menu:
            if(IsKeyPressed(KeyboardKey.Enter)){lives=MaxLives;turn=1;StartTurn();}
            break;

        case GameState.Ready:
        case GameState.Playing:
        {
            // Arrow keys rotate active camera
            if(IsKeyDown(KeyboardKey.Left))camYaw[activeCh]-=ArrowSpd*dt;
            if(IsKeyDown(KeyboardKey.Right))camYaw[activeCh]+=ArrowSpd*dt;

            // Map 0-9 to CH0-CH9 
            KeyboardKey[] camKeys = { KeyboardKey.Zero, KeyboardKey.One, KeyboardKey.Two, KeyboardKey.Three, KeyboardKey.Four, KeyboardKey.Five, KeyboardKey.Six, KeyboardKey.Seven, KeyboardKey.Eight, KeyboardKey.Nine };
            for(int i=0;i<camPos.Count&&i<10;i++) if(IsKeyPressed(camKeys[i]))activeCh=i;

            if (gs == GameState.Ready)
            {
                RenderMonitor(activeCh);
                if(IsKeyPressed(KeyboardKey.Y)){gs=GameState.Playing;turnTimer=TurnDur;graceTimer=0f;}
            }
            else // GameState.Playing
            {
                UpdateMonster(dt);
                monInView=IsMonsterVisible(activeCh);
                if(!monInView){graceTimer+=dt;if(graceTimer>=GracePer)TriggerScare();}
                else graceTimer=0f;
                turnTimer-=dt;
                if(turnTimer<=0f){turn++;if(turn>MaxTurns)gs=GameState.GameClear;else StartTurn();}
                RenderMonitor(activeCh);
            }
            break;
        }

        case GameState.Scare:
        {
            scareTimer+=dt;
            float ny=playerYaw%360f;
            if(ny>180f)ny-=360f;if(ny<-180f)ny+=360f;
            bool behind=MathF.Abs(ny)>140f;
            bool forward=MathF.Abs(ny)<40f;

            if(livesAtScare > 1)
            {
                bool inputLocked = false;
                if (curScareType == 0 && scarePhase >= 1) inputLocked = true;
                if (curScareType == 1 && scarePhase >= 3) inputLocked = true;
                if (curScareType == 2 && scarePhase >= 5) inputLocked = true;
                if (curScareType == 3 && scarePhase >= 1) inputLocked = true;
                if (curScareType == 4) inputLocked = true;
                if (curScareType == 5 && scarePhase >= 3) inputLocked = true;

                if (!inputLocked)
                {
                    if(IsKeyDown(KeyboardKey.Left))playerYaw-=ArrowSpd*dt;
                    if(IsKeyDown(KeyboardKey.Right))playerYaw+=ArrowSpd*dt;
                    if(IsKeyDown(KeyboardKey.Up))playerPitch=Math.Clamp(playerPitch+ArrowSpd*dt,-60,60);
                    if(IsKeyDown(KeyboardKey.Down))playerPitch=Math.Clamp(playerPitch-ArrowSpd*dt,-60,60);
                }
                else
                {
                    float targetY = playerYaw; float targetP = playerPitch;
                    
                    if (curScareType == 0) { targetY = playerYaw - ny + (ny > 0 ? 180f : -180f); targetP = 0f; }
                    if (curScareType == 1 || curScareType == 4 || curScareType == 5) { targetY = playerYaw - ny; targetP = 0f; }
                    if (curScareType == 2) { targetY = playerYaw; targetP = 70f; } // forced to look strictly up locally
                    if (curScareType == 3) { targetY = playerYaw; targetP = 0f; }
                    
                    playerYaw += (targetY - playerYaw) * 15f * dt;
                    playerPitch += (targetP - playerPitch) * 15f * dt;
                }
                UpdateScareCam();

                if(curScareType == 0)
                {
                    if(scarePhase==0 && behind){ scarePhase=1; scareTimer=0f; }
                    else if(scarePhase==1 && scareTimer>1.45f && !scareBlack){ scareBlack=true; blackTimer=0f; }
                }
                else if(curScareType == 1)
                {
                    if(scarePhase==0 && behind){scarePhase=1;scareTimer=0f;}
                    else if(scarePhase==1 && scareTimer>1.5f){scarePhase=2;scareTimer=0f;}
                    else if(scarePhase==2 && forward){scarePhase=3;scareTimer=0f;}
                    else if(scarePhase==3 && scareTimer>1.45f && !scareBlack){scareBlack=true;blackTimer=0f;}
                }
                else if (curScareType == 2)
                {
                    if(scarePhase==0 && behind){scarePhase=1;scareTimer=0f;}
                    else if(scarePhase==1 && scareTimer>1.2f){scarePhase=2;scareTimer=0f;} 
                    else if(scarePhase==2 && forward){scarePhase=3;scareTimer=0f;} 
                    else if(scarePhase==3 && scareTimer>1.2f){scarePhase=4;scareTimer=0f;}
                    else if(scarePhase==4 && behind){scarePhase=5;scareTimer=0f;} // auto-look up
                    else if(scarePhase==5 && scareTimer>1.8f && !scareBlack){scareBlack=true;blackTimer=0f;} 
                }
                else if (curScareType == 3)
                {
                    if(scarePhase==0 && scareTimer>3.5f && !scareBlack){scareBlack=true;blackTimer=0f;} 
                }
                else if (curScareType == 4)
                {
                    if(scarePhase==0 && scareTimer>1.5f){scarePhase=1;scareTimer=0f;} // wait, then monster dashes
                    else if(scarePhase==1 && scareTimer>0.8f && !scareBlack){scareBlack=true;blackTimer=0f;}
                }
                else if (curScareType == 5)
                {
                    if(scarePhase==0 && behind){scarePhase=1;scareTimer=0f;}
                    else if(scarePhase==1 && scareTimer>1.0f){scarePhase=2;scareTimer=0f;}
                    else if(scarePhase==2 && forward){scarePhase=3;scareTimer=0f;}
                    else if(scarePhase==3 && scareTimer>1.45f && !scareBlack){scareBlack=true;blackTimer=0f;}
                }

                if(scareBlack){blackTimer+=dt;if(blackTimer>BlackDur){playerYaw=0;playerPitch=0;StartTurn();}}
            }
            else if(livesAtScare==1) // ── SCARE 3: Corridor Chase ──
            {
                if(chaseIntroPhase==0) // Phase 0: Look back at approaching monster (Locked)
                {
                    UpdateScareCam();

                    chaseMonsterT+=chaseMonSpeed*dt;
                    if(chasePlayerT-chaseMonsterT<12f) // When monster gets to 12 tiles away
                    {
                        chaseIntroPhase=1; scareTimer=0f;
                    }
                }
                else if(chaseIntroPhase==1) // Phase 1: Auto 180 rotation & RUN prompt
                {
                    // Smoothly rotate to face FORWARD (0 deg or 360 deg)
                    float rotT = Math.Clamp(scareTimer / 0.6f, 0f, 1f);
                    float startRot = playerYaw;
                    float targetRot = startRot > 180f ? 360f : 0f;

                    float curYaw = scareInitYaw + startRot + rotT * (targetRot - startRot);
                    float yr = curYaw * MathF.PI / 180f;
                    scareCam.Target = scareCam.Position + new Vector3(MathF.Sin(yr), 0, -MathF.Cos(yr));

                    // Wait for Space to run
                    if (scareTimer > 1.2f && IsKeyPressed(KeyboardKey.Space))
                    {
                        chaseIntroPhase = 2; chaseRunning = true; scareTimer = 0f;
                        playerYaw = 0f; playerPitch = 0f; scareInitYaw = 0f;
                    }
                }
                else
                {
                    // Phase 2: Running! All controls unlocked
                    if(IsKeyDown(KeyboardKey.Left))playerYaw-=ArrowSpd*dt;
                    if(IsKeyDown(KeyboardKey.Right))playerYaw+=ArrowSpd*dt;
                    if(IsKeyDown(KeyboardKey.Up))playerPitch=Math.Clamp(playerPitch+ArrowSpd*dt,-60,60);
                    if(IsKeyDown(KeyboardKey.Down))playerPitch=Math.Clamp(playerPitch-ArrowSpd*dt,-60,60);
                    if(IsKeyDown(KeyboardKey.Space))chasePlayerT+=chaseSpeed*dt;
                    else chasePlayerT+=chaseSpeed*0.4f*dt;
                    chaseMonsterT+=chaseMonSpeed*dt;
                    scareCam.Position=new Vector3(600,1.6f,-chasePlayerT);
                    float cyr3=(scareInitYaw+playerYaw)*MathF.PI/180f;
                    float cpr3=playerPitch*MathF.PI/180f;
                    scareCam.Target=scareCam.Position+new Vector3(MathF.Sin(cyr3)*MathF.Cos(cpr3),MathF.Sin(cpr3),-MathF.Cos(cyr3)*MathF.Cos(cpr3));
                    if(chaseMonsterT>=chasePlayerT){scarePhase=1;if(scareTimer>1.5f)gs=GameState.GameOver;}
                    if(scarePhase==0&&chaseMonsterT>=chasePlayerT-1f)scarePhase=1;
                }
            }
            RenderMonitor(activeCh); // keep rendering bg monitor
            break;
        }

        case GameState.GameOver:
            if(IsKeyPressed(KeyboardKey.Enter)){lives=MaxLives;turn=1;gs=GameState.Menu;}
            break;
        case GameState.GameClear:
            if(IsKeyPressed(KeyboardKey.Enter)){lives=MaxLives;turn=1;gs=GameState.Menu;}
            break;
    }

    // ─── DRAW ─────────────────────────────────────────────────────────────────
    BeginDrawing();
    ClearBackground(C(8,8,12,255));

    switch(gs)
    {
        case GameState.Menu:
        {
            DrawRectangle(0,0,SW,SH,C(5,4,8,255));
            for(int i=0;i<10;i++)DrawRectangle(0,rng.Next(SH),SW,1,C(18,48,18,44));
            string t1="SECURITY MONITOR  SURVIVAL";
            DrawText(t1,(SW-MeasureText(t1,46))/2,SH/2-140,46,C(200,30,30,255));
            DrawText("Can you keep watching?",(SW-MeasureText("Can you keep watching?",22))/2,SH/2-78,22,C(160,160,160,220));
            int cx=(SW-520)/2,cy=SH/2-30;
            DrawRectangle(cx,cy,520,162,C(13,13,20,235));
            DrawRectangleLinesEx(new Rectangle(cx,cy,520,162),1,C(60,60,95,255));
            DrawText("CONTROLS",cx+14,cy+10,17,C(175,175,200,255));
            DrawText("[1]-[9]: Switch Camera",cx+14,cy+36,15,Color.White);
            DrawText("[Arrow L/R]: Rotate Camera Direction",cx+14,cy+58,15,Color.White);
            DrawText("RULES",cx+14,cy+84,17,C(200,165,78,255));
            DrawText("Keep the monster inside camera FOV!",cx+14,cy+108,14,Color.White);
            DrawText("Camera FOV is shown on minimap as yellow cone.",cx+14,cy+126,13,C(160,160,160,255));
            DrawText($"Turns: {MaxTurns}  Lives: {MaxLives}  Grace: {(int)GracePer}s",cx+14,cy+144,13,C(160,160,160,255));
            byte al=(byte)(140+(int)(MathF.Sin((float)GetTime()*3f)*110));
            DrawText("[ Press ENTER to Start ]",(SW-MeasureText("[ Press ENTER to Start ]",24))/2,SH/2+152,24,C(255,255,255,al));
            break;
        }

        case GameState.Ready:
        {
            DrawRectangle(0,0,SW,SH,C(6,6,10,255));
            DrawRectangle(0,0,SW,50,C(0,0,0,200));
            DrawText($"TURN {turn}/{MaxTurns}",14,13,22,Color.White);
            for(int i=0;i<MaxLives;i++)DrawText("*",SW-28-i*22,12,26,i<lives?C(230,50,50,255):C(65,30,30,255));
            // Monitor preview (top-left)
            int pW=440,pH=(int)(440f*MTH/MTW),pX=12,pY=56;
            DrawRectangle(pX-2,pY-2,pW+4,pH+4,C(0,0,0,255));
            DrawTexturePro(monRT.Texture,new(0,0,MTW,-MTH),new(pX,pY,pW,pH),Vector2.Zero,0f,Color.White);
            DrawRectangleLinesEx(new Rectangle(pX,pY,pW,pH),2,C(50,170,50,200));
            DrawText("CAMERA PREVIEW",pX,pY+pH+4,12,C(100,100,100,255));
            // Cam buttons below preview
            int bsY2=pY+pH+20;
            for(int i=0;i<camPos.Count&&i<10;i++){
                int bx=pX+i*56; bool sel=i==activeCh;
                DrawRectangle(bx,bsY2,52,32,sel?C(38,155,58,230):C(20,20,20,220));
                DrawRectangleLinesEx(new Rectangle(bx,bsY2,52,32),sel?2:1,sel?C(75,215,95,255):C(60,60,60,200));
                
                Color tc = C(150,150,150,255);
                if (camBroken[i]) tc = C(255,100,100,255);
                else if (sel) tc = Color.White;
                
                DrawText(camBroken[i]?"[x]":$"[{i}]",bx+4,bsY2+2,11,camBroken[i]?C(255,80,80,255):C(135,135,135,255));
                DrawText(camBroken[i]?"ERR":camName[i],bx+4,bsY2+16,12,tc);
            }

            // ── Map with FOV cones (right side) ──
            int mapX=pX+pW+16, mapY=56, mapW=SW-mapX-12, mapH=(int)(SH*0.62f);
            DrawRectangle(mapX-2,mapY-2,mapW+4,mapH+4,C(8,12,8,255));
            DrawRectangleLinesEx(new Rectangle(mapX-2,mapY-2,mapW+4,mapH+4),1,C(30,65,30,200));
            DrawText("CORRIDOR MAP + CAMERA FOV",mapX+4,mapY+4,13,C(75,195,75,255));
            // Compute map bounds
            if(path.Count>=2){
                float mnX=float.MaxValue,mxX=float.MinValue,mnZ=float.MaxValue,mxZ=float.MinValue;
                foreach(var p in path){if(p.X<mnX)mnX=p.X;if(p.X>mxX)mxX=p.X;if(p.Z<mnZ)mnZ=p.Z;if(p.Z>mxZ)mxZ=p.Z;}
                float rX=MathF.Max(mxX-mnX,1f),rZ=MathF.Max(mxZ-mnZ,1f);
                int innerX=mapX+8,innerY=mapY+22,innerW=mapW-16,innerH=mapH-30;
                float sc=MathF.Min(innerW/rX,innerH/rZ)*0.88f;
                float offX=innerX+(innerW-rX*sc)*0.5f,offZ=innerY+(innerH-rZ*sc)*0.5f;
                Vector2 MP(Vector3 v)=>new(offX+(v.X-mnX)*sc,offZ+(v.Z-mnZ)*sc);
                // Path
                for(int i=0;i<path.Count-1;i++)
                    DrawLineEx(MP(path[i]),MP(path[i+1]),5f,C(50,90,50,210));
                // FOV cones
                for(int ci=0;ci<camPos.Count;ci++){
                    Vector2 cp2=MP(camPos[ci]);
                    float ca=camYaw[ci]*MathF.PI/180f;
                    float hf=CamFov*0.5f*MathF.PI/180f;
                    float vd=MathF.Max(innerW,innerH)*0.7f;
                    bool sel=ci==activeCh;
                    Color fc2=sel?C(255,255,100,45):C(100,200,100,20);
                    for(int f=0;f<12;f++){
                        float a0=ca-hf+f*(CamFov*MathF.PI/180f/12f);
                        float a1=ca-hf+(f+1)*(CamFov*MathF.PI/180f/12f);
                        Vector2 q0=cp2+new Vector2(MathF.Sin(a0),-MathF.Cos(a0))*vd;
                        Vector2 q1=cp2+new Vector2(MathF.Sin(a1),-MathF.Cos(a1))*vd;
                        DrawTriangle(cp2,q0,q1,fc2);
                    }
                    Vector2 fL=cp2+new Vector2(MathF.Sin(ca-hf),-MathF.Cos(ca-hf))*vd;
                    Vector2 fR=cp2+new Vector2(MathF.Sin(ca+hf),-MathF.Cos(ca+hf))*vd;
                    DrawLineEx(cp2,fL,1.2f,sel?C(255,230,80,120):C(100,200,100,50));
                    DrawLineEx(cp2,fR,1.2f,sel?C(255,230,80,120):C(100,200,100,50));
                    Color cc2 = sel ? C(255,220,60,240) : C(125,195,125,175);
                    if (camBroken[ci]) cc2 = sel ? C(255,100,100,240) : C(200,50,50,175); // Broken UI
                    DrawCircleV(cp2,sel?6f:4f,cc2);
                    DrawText(camBroken[ci]?"X":camName[ci],(int)cp2.X+7,(int)cp2.Y-5,10,cc2);
                }
                // Monster
                Vector2 mm=MP(monPos);
                DrawCircleV(mm,6f,C(255,40,40,235));
                DrawCircleLinesV(mm,6f,Color.Red);
                DrawText("MON",(int)mm.X+7,(int)mm.Y-5,9,C(255,80,80,200));
            }

            // Legend below map
            int legY=mapY+mapH+8;
            DrawCircle(mapX+10,legY+8,5,C(255,220,60,240));DrawText("= Active Cam (Arrow L/R rotate)",mapX+22,legY+2,13,Color.White);
            DrawCircle(mapX+10,legY+28,4,C(125,195,125,175));DrawText("= Other Camera",mapX+22,legY+22,13,Color.White);
            DrawCircle(mapX+10,legY+48,5,C(255,40,40,235));DrawText("= Monster",mapX+22,legY+42,13,Color.White);
            DrawText("Yellow cone = Camera FOV",mapX+8,legY+62,12,C(200,200,100,200));
            DrawText("Cameras see STRAIGHT only (not around corners)",mapX+8,legY+78,11,C(150,150,150,200));

            byte al2=(byte)(140+(int)(MathF.Sin((float)GetTime()*4f)*110));
            DrawText("[ Press Y to Start Monitoring ]",(SW-MeasureText("[ Press Y to Start Monitoring ]",20))/2,SH-42,20,C(255,255,100,al2));
            break;
        }

        case GameState.Playing:
        {
            // Monitor (left)
            int mW=(int)(SW*0.62f),mH=(int)(mW*MTH/(float)MTW);
            DrawRectangle(-1,49,mW+2,mH+2,Color.Black);
            DrawTexturePro(monRT.Texture,new(0,0,MTW,-MTH),new(0,50,mW,mH),Vector2.Zero,0f,Color.White);
            DrawRectangleLinesEx(new Rectangle(0,50,mW,mH),3,C(26,75,26,200));
            // Right panel: minimap with FOV cones
            int rpX2=mW+8;
            DrawRectangle(rpX2-2,50,SW-rpX2+2,SH-50,C(5,8,5,255));
            DrawRectangleLinesEx(new Rectangle(rpX2-2,50,SW-rpX2+2,SH-50),1,C(25,52,25,200));
            DrawText("MAP",rpX2+4,56,14,C(70,190,70,255));
            int mmW2=SW-rpX2-12, mmH2=(int)(SH*0.55f);
            DrawRectangle(rpX2+4,74,mmW2,mmH2,C(8,14,8,255));
            DrawMinimapWithFOV(rpX2+4,74,mmW2,mmH2);
            // Status below
            int stY=74+mmH2+10;
            DrawText($"TURN: {turn}/{MaxTurns}",rpX2+6,stY,14,Color.White);
            DrawText($"IN FOV: {(monInView?"YES":"OUT")}",rpX2+6,stY+20,14,monInView?Color.Lime:Color.Red);
            DrawText($"GRACE: {(monInView?"--":$"{GracePer-graceTimer:F1}s")}",rpX2+6,stY+40,14,monInView?C(90,90,90,255):Color.Yellow);
            if(monTurnedBack)DrawText("!! Monster turned around !!",rpX2+6,stY+62,14,C(255,80,80,255));
            DrawHUD();
            break;
        }

        case GameState.Scare:
        {
            if(scareBlack){DrawRectangle(0,0,SW,SH,Color.Black);DrawText($"LIVES: {lives}",18,18,24,Color.Red);break;}

            if(livesAtScare > 1) 
            {
                BeginMode3D(scareCam);
                Vector3 roomBase=scareCam.Position-new Vector3(0,1.6f,0);
                DrawCube(roomBase,12f,0.1f,12f,C(25,22,28,255));
                DrawCube(roomBase+new Vector3(0,4f,0),12f,0.1f,12f,C(18,16,20,255));
                
                float fy=scareInitYaw*MathF.PI/180f;
                Vector3 monitorPos=scareCam.Position+new Vector3(MathF.Sin(fy),0,-MathF.Cos(fy))*2.5f;
                if (curScareType == 1 || curScareType == 2 || curScareType == 5) {
                    DrawCube(monitorPos-new Vector3(0,0.3f,0),1.8f,1.2f,0.15f,C(15,15,18,255));
                }

                if(curScareType == 0)
                {
                    if(scarePhase>=1) {
                        float yr=(scareInitYaw+180f)*MathF.PI/180f;
                        float approach = 0f;
                        if (scareTimer > 1.25f) approach = MathF.Min((scareTimer - 1.25f) * 12f, 2.8f);
                        Vector3 mb=scareCam.Position+new Vector3(MathF.Sin(yr),0,-MathF.Cos(yr))*(3.5f - approach)-new Vector3(0,0.7f,0);
                        DrawMonster(mb,scareInitYaw);
                    }
                }
                else if (curScareType == 1)
                {
                    if(scarePhase>=3) {
                        float approach = 0f;
                        if (scareTimer > 1.25f) approach = MathF.Min((scareTimer - 1.25f) * 12f, 2.0f);
                        Vector3 mpS=monitorPos+new Vector3(0,-0.5f,0) - new Vector3(MathF.Sin(fy),0,-MathF.Cos(fy))*approach;
                        DrawMonster(mpS,scareInitYaw+180f);
                    }
                }
                else if (curScareType == 2)
                {
                    if (scarePhase>=5) {
                        float approach = 0f;
                        if (scareTimer > 1.25f) approach = MathF.Min((scareTimer - 1.25f) * 15f, 2.8f);
                        float pyr = (scareInitYaw + playerYaw) * MathF.PI / 180f;
                        // Put monster directly above where player is looking
                        Vector3 monUp=scareCam.Position + new Vector3(MathF.Sin(pyr)*0.5f, 3.0f - approach, -MathF.Cos(pyr)*0.5f) - new Vector3(0,0.5f,0); 
                        DrawMonster(monUp, scareInitYaw + playerYaw + 180f);
                    }
                }
                else if (curScareType == 3)
                {
                    float dist = MathF.Max(3.0f - scareTimer * 0.8f, 0.45f); // stops closing in at 0.45f distance
                    for(int i=0; i<8; i++) {
                        float ry = (i * 45f) * MathF.PI / 180f;
                        Vector3 ms = scareCam.Position + new Vector3(MathF.Sin(ry),0,-MathF.Cos(ry))*dist - new Vector3(0,0.7f,0);
                        DrawMonster(ms, i * 45f + 180f);
                    }
                }
                else if (curScareType == 4)
                {
                    float approach = 0f;
                    if (scarePhase>=1) approach = MathF.Min(scareTimer * 15f, 2.0f);
                    Vector3 ms=scareCam.Position+new Vector3(MathF.Sin(fy),0,-MathF.Cos(fy))*(2.5f - approach)-new Vector3(0,0.7f,0);
                    DrawMonster(ms,scareInitYaw+180f);
                }
                else if (curScareType == 5)
                {
                    if(scarePhase==1) {
                        float yr=(scareInitYaw+180f)*MathF.PI/180f;
                        Vector3 mb=scareCam.Position+new Vector3(MathF.Sin(yr),0,-MathF.Cos(yr))*3.5f-new Vector3(0,0.7f,0);
                        DrawMonster(mb,scareInitYaw);
                    }
                    if(scarePhase>=3) {
                        float approach = 0f;
                        if (scareTimer > 1.25f) approach = MathF.Min((scareTimer - 1.25f) * 12f, 2.0f);
                        Vector3 mpS=monitorPos+new Vector3(0,-0.5f,0) - new Vector3(MathF.Sin(fy),0,-MathF.Cos(fy))*approach;
                        DrawMonster(mpS,scareInitYaw+180f);
                    }
                }
                EndMode3D();

                DrawRectangle(0,0,SW,46,C(0,0,0,180));
                DrawText($"LIVES: {lives}",14,12,20,Color.White);
                
                byte pA=(byte)(160+(int)(MathF.Sin((float)GetTime()*6f)*90));
                float ny2=playerYaw%360f;if(ny2>180f)ny2-=360f;if(ny2<-180f)ny2+=360f;
                string dirH=MathF.Abs(ny2)>90f?"Almost there...":ny2<0?"<<< Turn LEFT":"Turn RIGHT >>>";

                if (curScareType == 0 || curScareType == 5) {
                    if (scarePhase==0) {
                        string pm="Something is behind you... [Arrow Keys to turn]";
                        DrawText(pm,(SW-MeasureText(pm,22))/2,SH*3/4,22,C(255,180,50,pA));
                        DrawText(dirH,(SW-MeasureText(dirH,18))/2,SH*3/4+32,18,C(255,200,100,200));
                    }
                    if(curScareType == 5 && scarePhase==1) DrawText("...It vanished?!",(SW-MeasureText("...It vanished?!",28))/2,SH/2,28,C(200,200,200,255));
                    if(curScareType == 5 && scarePhase==2) DrawText("Turn back... [Arrow Keys]",(SW-MeasureText("Turn back... [Arrow Keys]",20))/2,SH*3/4,20,C(200,200,100,200));
                    if(curScareType==0 && scarePhase>=1 && scareTimer > 1.25f) DrawRectangle(0,0,SW,SH,C(150,0,0,100));
                    if(curScareType==5 && scarePhase>=3 && scareTimer > 1.25f) DrawRectangle(0,0,SW,SH,C(150,0,0,100));
                }
                else if (curScareType == 1) {
                    if (scarePhase==0) {
                        string pm="You hear something behind you... [Arrow Keys]";
                        DrawText(pm,(SW-MeasureText(pm,22))/2,SH*3/4,22,C(255,180,50,pA));
                        DrawText(dirH,(SW-MeasureText(dirH,18))/2,SH*3/4+32,18,C(255,200,100,200));
                    }
                    if(scarePhase==1) DrawText("...Nothing there.",(SW-MeasureText("...Nothing there.",28))/2,SH/2,28,C(200,200,200,255));
                    if(scarePhase==2) DrawText("Turn back... [Arrow Keys]",(SW-MeasureText("Turn back... [Arrow Keys]",20))/2,SH*3/4,20,C(200,200,100,200));
                    
                    if (scarePhase>=3) {
                        if (scareTimer > 1.25f) DrawRectangle(0,0,SW,SH,C(150,0,0,100));
                        DrawText("IT WAS WAITING",(SW-MeasureText("IT WAS WAITING",42))/2,SH/2-20,42,C(255,20,20,255));
                    }
                }
                else if (curScareType == 2) {
                    if (scarePhase==0) {
                        string pm="You hear something behind you... [Arrow Keys]";
                        DrawText(pm,(SW-MeasureText(pm,22))/2,SH*3/4,22,C(255,180,50,pA));
                        DrawText(dirH,(SW-MeasureText(dirH,18))/2,SH*3/4+32,18,C(255,200,100,200));
                    }
                    if(scarePhase==1) DrawText("...Nothing there.",(SW-MeasureText("...Nothing there.",28))/2,SH/2,28,C(200,200,200,255));
                    if(scarePhase==2) DrawText("Turn back... [Arrow Keys]",(SW-MeasureText("Turn back... [Arrow Keys]",20))/2,SH*3/4,20,C(200,200,100,200));
                    if(scarePhase==3) DrawText("...Still nothing there.",(SW-MeasureText("...Still nothing there.",28))/2,SH/2,28,C(200,200,200,255));
                    if(scarePhase==4) DrawText("Look behind you AGAIN... [Arrow Keys]",(SW-MeasureText("Look behind you AGAIN... [Arrow Keys]",24))/2,SH*3/4,24,C(255,100,100,200));
                    if (scarePhase>=5) {
                        if (scareTimer > 1.25f) { DrawRectangle(0,0,SW,SH,C(150,0,0,100)); DrawText("LOOK UP",(SW-MeasureText("LOOK UP",42))/2,SH/2-20,42,C(255,50,50,255)); }
                    }
                }
                else if (curScareType == 3) {
                    if(scarePhase==0) {
                        byte fA=(byte)Math.Min(255,(int)(scareTimer/2.5f*200));
                        DrawRectangle(0,0,SW,SH,C(60,10,10,fA));
                        string hm = "They are coming...";
                        DrawText(hm,(SW-MeasureText(hm,28))/2,SH/2,28,C(200,50,50,(byte)(fA)));
                    }
                }
                else if (curScareType == 4) {
                    if (scarePhase>=1) {
                         if (scareTimer > 0.3f) DrawRectangle(0,0,SW,SH,C(150,0,0,200)); 
                    }
                }
            }
            else // ── SCARE 3: Corridor chase ──
            {
                BeginMode3D(scareCam);
                // Procedural straight tube with stripes & obstacles
                int startT=(int)(chasePlayerT/Tile)-10;
                int endT=(int)(chasePlayerT/Tile)+50;
                for(int t=startT;t<endT;t++){
                    float z=-t*Tile;
                    bool stripe=(t%2==0);
                    Color flc=stripe?C(145,135,115,255):C(95,88,75,255);
                    Color wlc=stripe?C(165,155,140,255):C(120,112,100,255);
                    DrawCube(new(600,0,z),Tile,0.15f,Tile,flc);
                    DrawCube(new(600,HallH,z),Tile,0.15f,Tile,C(100,95,88,255));
                    DrawCube(new(600+Tile*0.5f,HallH*0.5f,z),0.15f,HallH,Tile,wlc);
                    DrawCube(new(600-Tile*0.5f,HallH*0.5f,z),0.15f,HallH,Tile,wlc);
                    if(t%5==0){DrawCube(new(600+Tile*0.48f,1.5f,z),0.08f,0.4f,Tile*0.8f,C(220,200,40,255));DrawCube(new(600-Tile*0.48f,1.5f,z),0.08f,0.4f,Tile*0.8f,C(220,200,40,255));}
                    if(t%7==3){float ox2=600+((t%3==0)?1.2f:-1.2f);DrawCylinder(new(ox2,0,z),0.35f,0.35f,1f,8,C(65,55,50,255));DrawCylinder(new(ox2,1f,z),0.37f,0.33f,0.05f,8,C(80,70,60,255));}
                    if(t%11==5)DrawCube(new(600,HallH-0.15f,z),Tile*0.6f,0.2f,0.2f,C(70,70,80,255));
                    if(t%4==0)DrawCube(new(600,HallH-0.08f,z),0.5f,0.08f,0.5f,C(240,235,200,200));
                }
                // Monster: eyes face player (chasing from behind, facing +Z toward player)
                Vector3 monChasePos2=new(600,0.5f,-chaseMonsterT);
                // Monster faces toward the player
                Vector3 toPlayer=scareCam.Position-monChasePos2;
                float monChaseYaw=MathF.Atan2(toPlayer.X,-toPlayer.Z)*180f/MathF.PI;
                DrawMonster(monChasePos2,monChaseYaw);
                EndMode3D();

                // Phase-specific overlays
                float dist3=chasePlayerT-chaseMonsterT;
                if(chaseIntroPhase==0) // Staring at monster coming closer
                {
                    DrawRectangle(0,0,SW,46,C(0,0,0,180));
                    DrawText("...What is that?",14,12,22,C(200,200,200,230));
                    // Heartbeat pulse
                    float pulse = MathF.Sin((float)GetTime()*4f)*0.5f+0.5f;
                    byte pA4=(byte)(int)(pulse*40);
                    DrawRectangle(0,0,SW,SH,C(40,0,0,pA4));
                }
                else if(chaseIntroPhase==1) // Auto-rotating, then showing RUN
                {
                    float rotP = Math.Clamp(scareTimer/0.6f,0f,1f);
                    if(rotP < 1f)
                    {
                        // During rotation: blur/panic
                        byte rotA=(byte)(int)((1f-rotP)*120);
                        DrawRectangle(0,0,SW,SH,C(0,0,0,rotA));
                    }
                    if(scareTimer > 0.6f)
                    {
                        // After rotation: big RUN text
                        byte rFlash=(byte)(200+(int)(MathF.Sin((float)GetTime()*8f)*55));
                        DrawText("R U N !",(SW-MeasureText("R U N !",64))/2,SH/2-50,64,C(255,30,30,rFlash));
                        if(scareTimer > 1.2f)
                            DrawText("[Press SPACE to run]",(SW-MeasureText("[Press SPACE to run]",22))/2,SH/2+30,22,C(255,255,200,220));
                    }
                }
                else // Running!
                {
                    float breathe2=MathF.Sin((float)GetTime()*6f)*0.3f+0.7f;
                    byte bA2=(byte)(int)(breathe2*60);
                    DrawRectangleGradientH(0,0,(int)(SW*0.3f),SH,C(20,0,0,bA2),C(0,0,0,0));
                    DrawRectangleGradientH((int)(SW*0.7f),0,(int)(SW*0.3f),SH,C(0,0,0,0),C(20,0,0,bA2));
                    // Crosshair with bob
                    float bob2=MathF.Sin((float)GetTime()*10f)*4f;
                    DrawRectangle(SW/2-1,(int)(SH/2-12+bob2),2,24,C(200,200,200,40));
                    DrawRectangle((int)(SW/2-12),SH/2-1,24,2,C(200,200,200,40));
                    DrawRectangle(0,0,SW,46,C(0,0,0,180));
                    bool holdingSpace=IsKeyDown(KeyboardKey.Space);
                    DrawText(holdingSpace?"RUNNING! [SPACE]":"Hold SPACE to run!",14,12,22,holdingSpace?C(50,255,80,255):C(255,140,40,255));
                    string distS2=$"Distance: {dist3:F1}m";
                    DrawText(distS2,SW-MeasureText(distS2,18)-14,16,18,dist3<3f?C(255,40,40,255):C(200,200,200,200));
                    if(dist3<3f){byte rA2=(byte)Math.Min(255,(int)((3f-dist3)/3f*200));DrawRectangle(0,0,SW,SH,C(80,0,0,rA2));}
                    if(scarePhase>=1)DrawText("G A M E   O V E R",(SW-MeasureText("G A M E   O V E R",52))/2,SH/2-30,52,C(255,0,0,255));
                }
            }
            break;
        }

        case GameState.GameOver:
        {
            DrawRectangle(0,0,SW,SH,C(8,0,0,255));
            for(int i=0;i<20;i++)DrawRectangle(rng.Next(SW),rng.Next(SH),rng.Next(50,SW),1,C(58,0,0,44));
            DrawText("G A M E   O V E R",(SW-MeasureText("G A M E   O V E R",56))/2,SH/2-75,56,Color.Red);
            DrawText("You lost sight of the monster.",(SW-MeasureText("You lost sight of the monster.",20))/2,SH/2+10,20,C(175,75,75,255));
            byte ra=(byte)(120+(int)(MathF.Sin((float)GetTime()*2f)*100));
            DrawText("[ Press ENTER to Restart ]",(SW-MeasureText("[ Press ENTER to Restart ]",22))/2,SH/2+90,22,C(255,255,255,ra));
            break;
        }

        case GameState.GameClear:
        {
            DrawRectangle(0,0,SW,SH,C(0,6,14,255));
            DrawText("S U R V I V E D",(SW-MeasureText("S U R V I V E D",52))/2,SH/2-90,52,C(48,198,98,255));
            DrawText("All shifts survived!",(SW-MeasureText("All shifts survived!",26))/2,SH/2-18,26,C(112,212,152,255));
            DrawText("Dawn has come.",(SW-MeasureText("Dawn has come.",18))/2,SH/2+24,18,C(92,152,115,220));
            byte rga=(byte)(120+(int)(MathF.Sin((float)GetTime()*2f)*100));
            DrawText("[ Press ENTER to Return to Menu ]",(SW-MeasureText("[ Press ENTER to Return to Menu ]",20))/2,SH/2+88,20,C(255,255,255,rga));
            break;
        }
    }

    EndDrawing();
}

UnloadRenderTexture(monRT);
CloseWindow();

enum GameState{Menu,Ready,Playing,Scare,GameOver,GameClear}
