#pragma once

#include <vector>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "core/frontend/emu_window.h"

struct ANativeWindow;

class EGLAndroid : public Frontend::EmuWindow {
public:
    EGLAndroid();
    ~EGLAndroid();

    bool Initialize(ANativeWindow* surface, float scale);

    std::unique_ptr<EGLAndroid> CreateSharedContext();
    void UpdateSurface(ANativeWindow* surface);
    void UpdateLayout();

    // EmuWindow implementation
    void SwapBuffers() override;
    void MakeCurrent() override;
    void DoneCurrent() override;
    void PollEvents() override;

private:
    EGLDisplay OpenEGLDisplay();
    EGLNativeWindowType GetEGLNativeWindow(EGLConfig config);

    bool CreateWindowSurface();
    void DestroyWindowSurface();
    void DestroyContext();

    float m_host_scale = 1.0f;
    ANativeWindow* m_new_window = nullptr;
    ANativeWindow* m_host_window = nullptr;

    unsigned window_width = 1;
    unsigned window_height = 1;

    bool m_is_shared = false;
    EGLConfig m_config;
    std::vector<int> m_attribs;
    EGLSurface m_egl_surface = EGL_NO_SURFACE;
    EGLContext m_egl_context = EGL_NO_CONTEXT;
    EGLDisplay m_egl_display = EGL_NO_DISPLAY;
};
