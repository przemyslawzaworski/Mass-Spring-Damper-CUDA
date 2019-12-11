#include <windows.h>

#define width 1920
#define height 1080

typedef unsigned int uint;

//////////////////////////////////////////////////////////////////////

__device__ float dot(float3 a, float3 b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

__device__ float length(float3 v)
{
	return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

__device__ float3 normalize(float3 v)
{
	float n = 1.0f / sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
	return  make_float3(n * v.x, n * v.y, n * v.z);
}

__device__ float3 reflect( float3 i, float3 n )
{
	float d = (n.x * i.x + n.y * i.y + n.z * i.z);
	return make_float3(i.x - 2.0f * n.x * d, i.y - 2.0f * n.y * d, i.z - 2.0f * n.z * d);
}

__device__ float clamp(float x, float a, float b)
{
	return fmaxf(a, fminf(b, x));
}

__device__ float4 texelFetch(float4 *sampler, uint x, uint y, int dx, int dy)  //fragCoord (x,y) and offset(dx,dy)
{
	uint q = (height - (y + dy) - 1) * width + (x + dx);
	return sampler[q];
}

__device__ float3 texelFetch3(float4 *sampler, uint x, uint y, int dx, int dy)
{
	uint q = (height - (y + dy) - 1) * width + (x + dx);
	float4 p = sampler[q];
	return make_float3(p.x, p.y, p.z);
}

//////////////////////////////////////////////////////////////////////

__global__ void BufferA(float4 *fragColor, float4 *buffer, float mx, float my)
{
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
	uint y = blockIdx.y * blockDim.y + threadIdx.y;
	uint id = (height - y - 1) * width + x;
	float kspring = 2.0f;
	float dspring = 1.0f;
	float mass = 20.0f;
	float dt = 1.0f;
	float2 mouse = make_float2(mx, my);
	float4 currentState = texelFetch(buffer, x, y, 0, 0);
	float force = 0.0f;
	float2 m = make_float2((x - mouse.x)*(x - mouse.x), (y - mouse.y)*(y - mouse.y));
	if (m.x < 5.f && m.y < 5.f) force -= 200.0f;           
	if (x<2 || x>width-2 || y<2 || y>height-2) return;
	for (int i=-1; i<=1; i+=2)
	{
		for (int j=-1; j<=1; j+=2)
		{
			float4 neighborState = texelFetch(buffer, x, y, i, j);
			if (x<4 || x>width-4 || y<4 || y>height-4) neighborState = make_float4(0.0f, 0.0f, 0.0f, 1.0f);
			float deltaP = neighborState.x - currentState.x;
			float deltaV = neighborState.y - currentState.y;
			force += kspring * deltaP + dspring * deltaV;
		}
	}
	float acceleration = force / mass;
	float velocity = acceleration * dt + currentState.y;
	float position = velocity * dt + currentState.x;
	fragColor[id] = make_float4(position, velocity, acceleration, 1.0f);
}

__global__ void BufferB(float4 *fragColor, float4 *buffer)
{
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
	uint y = blockIdx.y * blockDim.y + threadIdx.y;
	uint id = (height - y - 1) * width + x;
	float4 state = texelFetch(buffer, x, y, 0, 0);
	float position = saturate(state.x / 1024.0f + 0.5f);
	fragColor[id] = make_float4(0.0f, 0.3f, (255.0f*position)/255.0f + 0.5f, 1.0f);
}

__global__ void Image(uchar4 *fragColor, float4 *buffer)
{
	uint x = blockIdx.x * blockDim.x + threadIdx.x;
	uint y = blockIdx.y * blockDim.y + threadIdx.y;
	uint id = (height - y - 1) * width + x;
	if (x<2 || x>width-2 || y<2 || y>height-2) return;
	float a = length(texelFetch3(buffer, x, y,  1, 0));  
	float b = length(texelFetch3(buffer, x, y, -1, 0));  
	float c = length(texelFetch3(buffer, x, y,  0, 1));  
	float d = length(texelFetch3(buffer, x, y,  0,-1));  
	float3 normal = normalize(make_float3((a-b)*height, (c-d)*height, 5.0f));
	float3 light = normalize(make_float3(1.f,1.f,2.f));
	float diffuse = clamp(dot(normal,light),0.3f,1.0f);
	float specular = powf(clamp(dot(reflect(light,normal),make_float3(0.f,0.f,-1.f)),0.0f,1.0f), 32.0f);
	float4 color = make_float4(buffer[id].x*diffuse+specular, buffer[id].y*diffuse+specular, buffer[id].z*diffuse+specular, 1.0f);
	fragColor[id] = make_uchar4(saturate(color.z)*255, saturate(color.y)*255, saturate(color.x)*255, 255);
}

//////////////////////////////////////////////////////////////////////

static LRESULT CALLBACK WindowProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (uMsg==WM_CLOSE || uMsg==WM_DESTROY || (uMsg==WM_KEYDOWN && wParam==VK_ESCAPE))
	{
		PostQuitMessage(0); return 0;
	}
	else
	{
		return DefWindowProc(hWnd, uMsg, wParam, lParam);
	}
}

int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	ShowCursor(0);
	int exit = 0;
	MSG msg;
	WNDCLASS win = {CS_OWNDC|CS_HREDRAW|CS_VREDRAW, WindowProc, 0, 0, 0, 0, 0, (HBRUSH)(COLOR_WINDOW+1), 0, "CUDA Demo"};
	RegisterClass(&win);
	HDC hdc = GetDC(CreateWindowEx(0, win.lpszClassName, "CUDA Demo", WS_VISIBLE|WS_POPUP, 0, 0, width, height, 0, 0, 0, 0));
	float4 *bufferA, *bufferB;
	uchar4 *image; 
	cudaMalloc( (void**)&bufferA, width*height*sizeof(float4) );
	cudaMalloc( (void**)&bufferB, width*height*sizeof(float4) );
	cudaMalloc( (void**)&image, width*height*sizeof(uchar4) );
	dim3 block(8, 8);
	dim3 grid(width/8, height/8);
	BITMAPINFO bmi = {{sizeof(BITMAPINFOHEADER),width,height,1,32,BI_RGB,0,0,0,0,0},{0,0,0,0}};
	static unsigned char host[width*height*4];
	POINT point;
	DWORD S = GetTickCount();
	while (!exit)
	{
		while(PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
		{
			if( msg.message==WM_QUIT ) exit = 1;
			TranslateMessage( &msg );
			DispatchMessage( &msg );
		}
		if (((GetTickCount() - S) % 3 == 0) && (GetAsyncKeyState(VK_LBUTTON)& 0x8000)) GetCursorPos(&point);
		BufferA<<<grid, block>>>(bufferA, bufferA, point.x, point.y);
		BufferB<<<grid, block>>>(bufferB, bufferA);
		Image<<<grid, block>>>(image, bufferB);
		cudaMemcpy(host, image, width * height * sizeof(uchar4), cudaMemcpyDeviceToHost);
		StretchDIBits(hdc,0,0,width,height,0,0,width,height,host,&bmi,DIB_RGB_COLORS,SRCCOPY);
	}
	cudaFree(bufferA);
	cudaFree(bufferB);
	cudaFree(image);
	return 0;
}