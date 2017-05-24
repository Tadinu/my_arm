/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include <tchar.h>
#include <math.h>
#include "util_render_D3D.h"
#include "pxcmetadata.h"
#include <Mfapi.h>

/// Used by m_scale
static const float DEFAULT_SCALE = 30.0f;

//TODO: distance must be based on camera properties
static const float MAX_Z = 0.5f / 13.0f;

UtilRenderD3D::UtilRenderD3D(pxcCHAR *title, PXCSession *session) {

	WNDCLASSW wc;
	memset(&wc, 0, sizeof(wc));
	wc.lpszClassName = title;
	wc.lpfnWndProc = WindowProc;
	wc.style = CS_HREDRAW | CS_VREDRAW;
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	RegisterClassW(&wc);

	m_bitmap = 0;
	m_fps_nframes = 0;
	wcscpy_s<1024>(m_title, title);
	m_frame = 0;
	m_space = 0;
	m_lastRenderedFPS = 0;

	m_hWnd = CreateWindowW(title, title, WS_BORDER | WS_CAPTION | WS_SYSMENU | WS_SIZEBOX, CW_USEDEFAULT, CW_USEDEFAULT, 100, 100, 0, 0, 0, this);
	m_pD3D = NULL;
	m_pDevice = NULL;
	m_pSwapChain = NULL;

	memset(&m_info, 0, sizeof(m_info));
	m_buffer = 0;
	m_scale = DEFAULT_SCALE;
	m_mouse.x = m_mouse.y = -1;
	m_depth_mode = RENDER_DEFAULT;
}

UtilRenderD3D::~UtilRenderD3D(void) {
	if (m_hWnd) DestroyWindow(m_hWnd);
	if (m_bitmap) DeleteObject(m_bitmap);
	if (m_buffer) free(m_buffer);
	if (m_pSwapChain) m_pSwapChain->Release();
	if (m_pDevice) m_pDevice->Release();
	if (m_pD3D) m_pD3D->Release();
}

void UtilRenderD3D::SetSize(pxcI32 width, pxcI32 height) {
	if (m_buffer) free(m_buffer);
	m_buffer = (pxcI32*)malloc(sizeof(pxcI32)*width*height);

	m_info.bmiHeader.biWidth = width;
	m_info.bmiHeader.biHeight = -height;
	m_info.bmiHeader.biBitCount = 32;
	m_info.bmiHeader.biPlanes = 1;
	m_info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	m_info.bmiHeader.biCompression = BI_RGB;

	// fit into the client rect
	WINDOWINFO winInfo;
	GetWindowInfo(m_hWnd, &winInfo);
	int borderX = (winInfo.rcWindow.right - winInfo.rcWindow.left) - (winInfo.rcClient.right - winInfo.rcClient.left);
	int borderY = (winInfo.rcWindow.bottom - winInfo.rcWindow.top) - (winInfo.rcClient.bottom - winInfo.rcClient.top);
	SetWindowPos(m_hWnd, 0, 0, 0, width + borderX, height + borderY, SWP_NOMOVE | SWP_NOZORDER | SWP_NOREDRAW | SWP_SHOWWINDOW);
}

static pxcI32 Float2Color(float value)
{
	pxcI32 color = 0xff000000;
	if (value <= 0.f || value > 1.f) return color;

	float saturation = 0.3f + 0.7f*(1.f - value);
	float hue = value * 6; // hue
	float V = 1.f - pow(value, 3); // brightness
	float F = hue - (int)hue;
	float M = V*(1 - saturation);
	float N = V*(1 - saturation*F);
	float K = V*(1 - saturation*(1 - F));
	float r, g, b;
	switch ((int)hue) {
	case 0: r = V; g = K; b = M; break;
	case 1: r = N; g = V; b = M; break;
	case 2: r = M; g = V; b = K; break;
	case 3: r = M; g = N; b = V; break;
	case 4: r = K; g = M; b = V; break;
	case 5: r = V; g = M; b = N; break;
	default: r = g = b = 0;
	}

	unsigned char* rgb = (unsigned char*)&color;
	rgb[0] = (unsigned char)(255.f*r);
	rgb[1] = (unsigned char)(255.f*g);
	rgb[2] = (unsigned char)(255.f*b);
	return color;
}

void SolidEdges(pxcI16* pSrc, int channels, pxcI32* pDst, int w, int h, float scale);
/*{
	pxcI16* src = pSrc + (channels-1);
	pxcI32* dst = pDst;

	float minZ = 0;
	float maxZ = MAX_Z;

	for(int i=0;i<h;i++) {
		for(int j=0;j<w;j++) {
			float z0 = (float)src[0] / 32768;
			float z1 = j>0   ? (float)src[-channels] / 32768 : 0;
			float z2 = j<w-1 ? (float)src[ channels] / 32768 : 0;
			float z3 = i>0   ? (float)src[-w*channels] / 32768 : 0;
			float z4 = i<h-1 ? (float)src[ w*channels] / 32768 : 0;
			if (abs(z0-z1)>scale || abs(z0-z2)>scale || abs(z0-z3)>scale || abs(z0-z4)>scale) {
				*dst = 0;
			} else {
				float val = (z0-minZ)/(maxZ-minZ);
				*dst = Float2Color(val);
			}
			src+=channels;
			dst++;
		}
	}
}
*/
void ConfidenceMap(pxcI16* pSrc, pxcI32* pDst, int w, int h);
/*{
	pxcI16* src = pSrc;
	pxcI32* dst = pDst;

	float minC = 1000.0f;
	float maxC = -1000.0f;

	for(int k=0;k<w*h;k++)
	{
		float vC = (float)src[0] / 32768;
		if (minC>vC) minC = vC;
		if (maxC<vC) maxC = vC;
		src++;
	}

	src = pSrc;
	for(int i=0;i<w*h;i++)
	{
		float val = (float)src[0] / 32768;
		val = (val-minC)/(maxC-minC);
		unsigned char* rgb = (unsigned char*)dst;
		rgb[0] = (unsigned char)(255.f*val);
		rgb[1] = (unsigned char)(255.f*val);
		rgb[2] = (unsigned char)(255.f*val);
		src++;
		dst++;
	}
}
*/
bool UtilRenderD3D::RenderFrame(PXCImage *image)
{
	if (!image) return false;

	// Lazy initialization of D3D
	if (!m_pSwapChain)
	{
		// Create the Direct3D context object
		m_pD3D = Direct3DCreate9(D3D_SDK_VERSION);
		if (m_pD3D != NULL)
		{
			// Create a Direct3D device object
			D3DDISPLAYMODE mode = { 0 };
			if (!FAILED(m_pD3D->GetAdapterDisplayMode(D3DADAPTER_DEFAULT, &mode)))
			{
				if (!FAILED(m_pD3D->CheckDeviceType(
					D3DADAPTER_DEFAULT,
					D3DDEVTYPE_HAL,
					mode.Format,
					D3DFMT_R8G8B8,
					TRUE    // windowed
					)))
				{
					D3DPRESENT_PARAMETERS pp = { 0 };
					pp.BackBufferFormat = D3DFMT_X8R8G8B8;
					pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
					pp.Windowed = TRUE;
					pp.hDeviceWindow = m_hWnd;
					PXCImage::ImageInfo info = image->QueryInfo();
					pp.BackBufferWidth = info.width;
					pp.BackBufferHeight = info.height;
					pp.SwapEffect = D3DSWAPEFFECT_FLIP;
					pp.Flags = D3DPRESENTFLAG_VIDEO | D3DPRESENTFLAG_DEVICECLIP |
						D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
					pp.BackBufferCount = 1;
					if (!FAILED(m_pD3D->CreateDevice(
						D3DADAPTER_DEFAULT,
						D3DDEVTYPE_HAL,
						m_hWnd,
						D3DCREATE_PUREDEVICE,
						&pp,
						&m_pDevice
						)))
					{
						// Create the swap chain
						m_pDevice->CreateAdditionalSwapChain(&pp, &m_pSwapChain);
					}
				}
			}
		}
	}

	/* Calcualte FPS and display it in the window title (frame) */
	if ((m_fps_nframes++) == 0) {
		m_rdtsc_first = __rdtsc();
		LARGE_INTEGER now, freq;
		QueryPerformanceCounter(&now);
		QueryPerformanceFrequency(&freq);
		m_time_first = now.QuadPart;
		m_freq = (double)freq.QuadPart;
	}
	if (m_fps_nframes > 30) {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		int fps = (int)((double)m_fps_nframes*(double)m_freq / (double)(now.QuadPart - m_time_first));
		m_lastRenderedFPS = fps;
#define UPDATE_FPS_IN_TITLE
#ifdef UPDATE_FPS_IN_TITLE
		pxcCHAR line[1024];
		swprintf_s<1024>(line, L"%s (%d fps)", m_title, fps);
		SetWindowTextW(m_hWnd, line);
#endif
		m_fps_nframes = 0;
	}

	if (!m_space) m_frame++;

	if (m_bitmap) {
		DeleteObject(m_bitmap);
	}

	PXCImage::ImageInfo info = image->QueryInfo();
	if ((info.width != (pxcI32)abs(m_info.bmiHeader.biWidth)) || (info.height != (pxcI32)abs(m_info.bmiHeader.biHeight)))
	{
		SetSize(info.width, info.height);
	}

	// Show the provided image
	if (m_pSwapChain) // Using D3D (if initialized)
	{
		IDirect3DSurface9* pSurf = NULL;
		D3DLOCKED_RECT lr;
		// Copy the image data to the back buffer
		if (!FAILED(m_pSwapChain->GetBackBuffer(0, D3DBACKBUFFER_TYPE_MONO, &pSurf))
			&& pSurf && !FAILED(pSurf->LockRect(&lr, NULL, D3DLOCK_NOSYSLOCK)))
		{
			PXCImage::ImageInfo info = image->QueryInfo();
			PXCImage::ImageData data;
			pxcStatus sts = image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &data);
			if (sts >= PXC_STATUS_NO_ERROR) {
				MFCopyImage((BYTE*)lr.pBits, lr.Pitch, data.planes[0], data.pitches[0], info.width * 3, info.height);
				image->ReleaseAccess(&data);
			}
			pSurf->UnlockRect();
		}

		// Color fill the back buffer.
		IDirect3DSurface9* pBB = NULL;
		m_pDevice->GetBackBuffer(0, 0, D3DBACKBUFFER_TYPE_MONO, &pBB);

		// Blit the frame.
		m_pDevice->StretchRect(pSurf, NULL, pBB, NULL, D3DTEXF_POINT);
		m_pDevice->Present(NULL, NULL, NULL, NULL);
		if (pBB) pBB->Release();
		if (pSurf) pSurf->Release();
	}
	else
	{
		if (m_buffer) {
			PXCImage::ImageInfo info = image->QueryInfo();
			PXCImage::ImageData data;
			//TODO give distance for the camera properties
			float max_distance = MAX_Z;
			float range_min = .0f;
			float range_max = max_distance * m_scale / DEFAULT_SCALE;

			if (m_depth_mode == RENDER_X || m_depth_mode == RENDER_Y || m_depth_mode == RENDER_Z)
			{
#if 0
				pxcStatus sts = projection->QueryVertices();
				if (sts >= PXC_STATUS_NO_ERROR)
				{
					pxcI32* dst = m_buffer;
					pxcF32* src = (pxcF32*)data.planes[0];
					int offset;
					switch (m_depth_mode)
					{
					case RENDER_X: // X coordinate in color
					case RENDER_Y: // Y coordinate in color
						offset = m_depth_mode == RENDER_X ? 0 : 1;
						for (int j = 0; j < -m_info.bmiHeader.biHeight * m_info.bmiHeader.biWidth; j++)
						{
							float val = src[offset] / 32768;
							val = (val - range_min) / (range_max - range_min);
							*dst = Float2Color(abs(val));
							dst++;
							src += 3;
						}
						break;
					case RENDER_Z: // Z coordinate in gray scale
						for (int j = 0; j < -m_info.bmiHeader.biHeight * m_info.bmiHeader.biWidth; j++)
						{
							float val = src[2] / 32768;
							val = 1.f - (val - range_min) / (range_max - range_min);
							val = max(val, 0.f);
							val = min(val, 1.f);
							val *= val; // use square 
							unsigned char* rgb = (unsigned char*)dst;
							rgb[0] = (unsigned char)(255.f*val);
							rgb[1] = (unsigned char)(255.f*val);
							rgb[2] = (unsigned char)(255.f*val);
							rgb[3] = 255;
							dst++;
							src += 3;
						}
						break;
					}
					sts = image->ReleaseAccess(&data);
				}
#endif
			}
			else if (m_depth_mode == RENDER_EDGES || m_depth_mode == RENDER_CONFIDENCE)
			{
#if 0
				// This implementation forces distance plane data to only render distance values
				// Vertex data can still render confidence and edges
				pxcStatus sts = image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &data);
				if (sts >= PXC_STATUS_NO_ERROR) {
					for (int i = 0;i < -m_info.bmiHeader.biHeight;i++)
						memcpy_s((char*)(m_buffer + i*m_info.bmiHeader.biWidth), m_info.bmiHeader.biWidth * 4, data.planes[0] + i*data.pitches[0], m_info.bmiHeader.biWidth * 4);
					sts = image->ReleaseAccess(&data);
				}
#else
				pxcStatus sts = image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &data);
				if (sts >= PXC_STATUS_NO_ERROR)
				{
					pxcI32* dst = m_buffer;
					pxcI16* src = (pxcI16*)data.planes[0];
					switch (m_depth_mode)
					{
					case RENDER_CONFIDENCE: // IR in gray scale
						if (data.planes[1])
						{
							ConfidenceMap((pxcI16*)data.planes[1], dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight);
							break;
						}
						m_depth_mode = RENDER_EDGES;
					case RENDER_EDGES: // Z coordinate in color with edge detection
						SolidEdges(src, 1, dst, m_info.bmiHeader.biWidth, -m_info.bmiHeader.biHeight, m_scale*0.001f*max_distance);
						break;
					}
					sts = image->ReleaseAccess(&data);
				}
#endif
			}
			else
			{
				pxcStatus sts = image->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &data);
				if (sts >= PXC_STATUS_NO_ERROR) {
					for (int i = 0; i < -m_info.bmiHeader.biHeight; i++)
						memcpy_s((char*)(m_buffer + i*m_info.bmiHeader.biWidth), m_info.bmiHeader.biWidth * 4, data.planes[0] + i*data.pitches[0], m_info.bmiHeader.biWidth * 4);
					sts = image->ReleaseAccess(&data);
				}
			}

			{
				HDC dc = GetDC(m_hWnd);
				m_bitmap = CreateDIBitmap(dc, &m_info.bmiHeader, CBM_INIT, m_buffer, &m_info, DIB_RGB_COLORS);
				ReleaseDC(m_hWnd, dc);
				InvalidateRect(m_hWnd, 0, FALSE);
			}
		}
	}

	DoMessageLoop();

	return IsWindowVisible(m_hWnd) ? true : false;
}

void UtilRenderD3D::DoMessageLoop(void) {
	for (MSG msg; PeekMessage(&msg, m_hWnd, 0, 0, PM_REMOVE);) {
		DispatchMessage(&msg);
	}
}

LRESULT UtilRenderD3D::WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
	UtilRenderD3D *pthis = (UtilRenderD3D*)GetWindowLongPtr(hwnd, GWLP_USERDATA);

	switch (uMsg) {
	case WM_CREATE:
		SetWindowLongPtr(hwnd, GWLP_USERDATA, (LONG_PTR)((CREATESTRUCT*)lParam)->lpCreateParams);
		return TRUE;
	case WM_SIZE:
		pthis->m_pSwapChain = NULL;
		return TRUE;
	case WM_PAINT:
		if (NULL == pthis->m_pSwapChain)
		{
			if (pthis->m_bitmap) {
				PAINTSTRUCT ps;
				HDC hdc = BeginPaint(hwnd, &ps);
				HDC hdc2 = CreateCompatibleDC(hdc);
				SelectObject(hdc2, pthis->m_bitmap);
				RECT rect;
				HFONT hFont = CreateFont(8, 0, 0, 0, FW_LIGHT, 0, 0, 0, 0, 0, 0, 2, 0, L"MONOSPACE");
				SetBkMode(hdc, TRANSPARENT);
				SetTextColor(hdc, RGB(255, 255, 255));
				SelectObject(hdc, hFont);
				GetClientRect(hwnd, &rect);
				SetStretchBltMode(hdc, HALFTONE); // slow but better quality
				//SetStretchBltMode(hdc, BLACKONWHITE); //Very poor quality(red stripes) in case of resizing. Bug 44556.
				StretchBlt(hdc, 0, 0, rect.right - rect.left, rect.bottom - rect.top, hdc2, 0, 0, pthis->m_info.bmiHeader.biWidth, -pthis->m_info.bmiHeader.biHeight, SRCCOPY);
				DeleteObject(hFont);
				if (hdc2)
					DeleteObject(hdc2); // Calling DeleteObject instead of DeleteDC since DeleteDC results in a GDI resource leak.
				pthis->DrawMore(hdc, (double)(rect.right - rect.left) / pthis->m_info.bmiHeader.biWidth, (double)(rect.bottom - rect.top) / (-pthis->m_info.bmiHeader.biHeight));
				EndPaint(hwnd, &ps);
			}
		}
		return TRUE;
	case WM_MOUSEWHEEL:
		if (GET_WHEEL_DELTA_WPARAM(wParam) > 0) pthis->m_scale *= 1.5; else pthis->m_scale /= 1.5;
		return TRUE;
	case WM_MOVE:
		pthis->m_fps_nframes = 0;
		return TRUE;
	case WM_LBUTTONDOWN:
		pthis->m_mouse.x = LOWORD(lParam);
		pthis->m_mouse.y = HIWORD(lParam);
		return FALSE;
	case WM_KEYDOWN:
		switch (wParam)
		{
		case VK_F1: // X coordinates in color (for vertices windows)
			pthis->m_depth_mode = RENDER_X;
			break;
		case VK_F2: // Y coordinates in color (for vertices windows)
			pthis->m_depth_mode = RENDER_Y;
			break;
		case VK_F3: // Z coordinates in gray scale (for vertices windows)
			pthis->m_depth_mode = RENDER_Z;
			pthis->m_scale = DEFAULT_SCALE;
			break;
		case VK_F5: // Z coordinate in color with edge detection
			pthis->m_depth_mode = RENDER_EDGES;
			pthis->m_scale = DEFAULT_SCALE;
			break;
		case VK_F6: // IR in gray scale
			pthis->m_depth_mode = RENDER_CONFIDENCE;
			break;
		case VK_RIGHT: // right - next frame
			pthis->m_frame++;
			break;
		case VK_LEFT: // left - previous frame
			pthis->m_frame--;
			if (pthis->m_frame < 0) pthis->m_frame = 0;
			break;
		case VK_SPACE: // space - unpause
			pthis->m_space = !pthis->m_space;
			break;
		}
		return FALSE;
	case WM_CLOSE:
		ShowWindow(hwnd, SW_HIDE);
		return FALSE;
	default:
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}
}

int UtilRenderD3D::GetCurrentFPS()
{
	return m_lastRenderedFPS;
}
