#include "egl_android.h"

#include <android/native_window_jni.h>
#include <glad/glad.h>

#include "video_core/renderer_base.h"
#include "video_core/video_core.h"

static constexpr std::array<EGLint, 15> egl_attribs{EGL_SURFACE_TYPE,
                                                    EGL_WINDOW_BIT,
                                                    EGL_RENDERABLE_TYPE,
                                                    EGL_OPENGL_ES3_BIT_KHR,
                                                    EGL_BLUE_SIZE,
                                                    8,
                                                    EGL_GREEN_SIZE,
                                                    8,
                                                    EGL_RED_SIZE,
                                                    8,
                                                    EGL_DEPTH_SIZE,
                                                    0,
                                                    EGL_STENCIL_SIZE,
                                                    0,
                                                    EGL_NONE};
static constexpr std::array<EGLint, 5> egl_empty_attribs{EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
static constexpr std::array<EGLint, 4> egl_context_attribs{EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};

class SharedContext_Android {
public:
    SharedContext_Android(EGLDisplay egl_display, EGLConfig egl_config, EGLContext egl_share_context)
            : egl_display{egl_display},
              egl_surface{eglCreatePbufferSurface(egl_display, egl_config, egl_empty_attribs.data())},
              egl_context{eglCreateContext(egl_display, egl_config, egl_share_context, egl_context_attribs.data())} {
    }

    ~SharedContext_Android() {
        if (!eglDestroySurface(egl_display, egl_surface)) {
            LOG_CRITICAL(Frontend, "eglDestroySurface() failed");
        }

        if (!eglDestroyContext(egl_display, egl_context)) {
            LOG_CRITICAL(Frontend, "eglDestroySurface() failed");
        }
    }

    void MakeCurrent() {
        eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
    }

    void DoneCurrent() {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }

private:
    EGLDisplay egl_display;
    EGLSurface egl_surface;
    EGLContext egl_context;
};

EGLAndroid::EGLAndroid(bool use_shared_context) : use_shared_context(use_shared_context) {}

bool EGLAndroid::Initialize(ANativeWindow* surface) {
    host_window = surface;
    egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (!egl_display) {
        // Error: eglGetDisplay() failed
        return false;
    }

    EGLint egl_major, egl_minor;
    if (!eglInitialize(egl_display, &egl_major, &egl_minor)) {
        // Error: eglInitialize() failed
        return false;
    }

    EGLint num_configs;
    if (!eglChooseConfig(egl_display, egl_attribs.data(), &egl_config, 1, &num_configs)) {
        // Error: couldn't get an EGL visual config
        return false;
    }

    egl_context = eglCreateContext(egl_display, egl_config, 0, egl_context_attribs.data());
    if (egl_context == EGL_NO_CONTEXT) {
        return false;
    }

    if (use_shared_context) {
        core_context = std::make_unique<SharedContext_Android>(egl_display, egl_config, egl_context);
    } else {
        presenting_state = PresentingState::Stopped;
    }

    CreateWindowSurface();

    return gladLoadGLES2Loader((GLADloadproc)eglGetProcAddress);
}

void EGLAndroid::UpdateSurface(ANativeWindow* surface) {
    new_window = surface;
    StopPresenting();
}

void EGLAndroid::UpdateLayout() {
    UpdateCurrentFramebufferLayout(window_width, window_height);
}

void EGLAndroid::CreateWindowSurface() {
    if (!host_window) {
        return;
    }
    EGLint format;
    eglGetConfigAttrib(egl_display, egl_config, EGL_NATIVE_VISUAL_ID, &format);
    ANativeWindow_setBuffersGeometry(host_window, 0, 0, format);
    egl_surface = eglCreateWindowSurface(egl_display, egl_config, host_window, nullptr);
    eglQuerySurface(egl_display, egl_surface, EGL_WIDTH, &window_width);
    eglQuerySurface(egl_display, egl_surface, EGL_HEIGHT, &window_height);
    eglSurfaceAttrib(egl_display, egl_surface, EGL_SWAP_BEHAVIOR, EGL_BUFFER_DESTROYED);
    MakeCurrent();
    UpdateLayout();
}

void EGLAndroid::DestroyWindowSurface() {
    if (!egl_surface) {
        return;
    }
    if (eglGetCurrentSurface(EGL_DRAW) == egl_surface) {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    if (!eglDestroySurface(egl_display, egl_surface)) {
        // Could not destroy window surface
    }
    egl_surface = EGL_NO_SURFACE;
}

void EGLAndroid::DestroyContext() {
    if (!egl_context) {
        return;
    }
    if (eglGetCurrentContext() == egl_context) {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    if (!eglDestroyContext(egl_display, egl_context)) {
        // Could not destroy drawing context
    }
    if (!eglTerminate(egl_display)) {
        // Could not destroy display connection
    }
    egl_context = EGL_NO_CONTEXT;
    egl_display = EGL_NO_DISPLAY;
}

EGLAndroid::~EGLAndroid() {
    DestroyWindowSurface();
    DestroyContext();
}

void EGLAndroid::TryPresenting() {
    if (presenting_state != PresentingState::Running) {
        if (presenting_state == PresentingState::Initial) {
            eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
            presenting_state = PresentingState::Running;
        } else {
            return;
        }
    }
    if (VideoCore::g_renderer->TryPresent()) {
        eglSwapBuffers(egl_display, egl_surface);
    }
}

void EGLAndroid::StopPresenting() {
    if (presenting_state == PresentingState::Running) {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    presenting_state = PresentingState::Stopped;
}

void EGLAndroid::SwapBuffers() {
    eglSwapBuffers(egl_display, egl_surface);
}

void EGLAndroid::PollEvents() {
    if (!new_window) {
        return;
    }

    host_window = new_window;
    new_window = nullptr;
    DestroyWindowSurface();
    CreateWindowSurface();
    VideoCore::g_renderer->ResetPresent();
    if (use_shared_context) {
        presenting_state = PresentingState::Initial;
    }
}

void EGLAndroid::MakeCurrent() {
    if (use_shared_context) {
        core_context->MakeCurrent();
    } else {
        eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);
    }
}

void EGLAndroid::DoneCurrent() {
    if (use_shared_context) {
        core_context->DoneCurrent();
    } else {
        eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
}
