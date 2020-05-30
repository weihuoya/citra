#pragma once

#include <vector>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "core/frontend/emu_window.h"

struct ANativeWindow;
class SharedContext_Android;

class EGLAndroid : public Frontend::EmuWindow {
public:
    explicit EGLAndroid(bool use_shared_context);
    ~EGLAndroid();

    bool Initialize(ANativeWindow* surface);
    void UpdateSurface(ANativeWindow* surface);
    void UpdateLayout();

    void MakeCurrent() override;
    void DoneCurrent() override;
    void PollEvents() override;
    void SwapBuffers() override;

    void TryPresenting();
    void StopPresenting();

private:
    void CreateWindowSurface();
    void DestroyWindowSurface();
    void DestroyContext();

    ANativeWindow* new_window = nullptr;
    ANativeWindow* host_window = nullptr;

    EGLint window_width = 1;
    EGLint window_height = 1;

    EGLConfig egl_config = nullptr;
    EGLSurface egl_surface = EGL_NO_SURFACE;
    EGLContext egl_context = EGL_NO_CONTEXT;
    EGLDisplay egl_display = EGL_NO_DISPLAY;

    std::unique_ptr<SharedContext_Android> core_context;

    enum class PresentingState {
        Initial,
        Running,
        Stopped,
    };
    PresentingState presenting_state{};
    bool use_shared_context;
};
