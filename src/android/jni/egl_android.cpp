#include "egl_android.h"

#include <android/native_window_jni.h>

#include <glad/glad.h>

static void* EGL_GetFuncAddress(const char* name) {
    return (void*)eglGetProcAddress(name);
}

EGLAndroid::EGLAndroid() = default;

EGLAndroid::~EGLAndroid() {
    DestroyWindowSurface();
    DestroyContext();
}

bool EGLAndroid::Initialize(ANativeWindow* surface, float scale) {
    m_host_scale = scale;
    m_host_window = surface;
    m_egl_display = OpenEGLDisplay();

    EGLint num_configs;
    EGLint egl_major, egl_minor;

    if (!m_egl_display) {
        // Error: eglGetDisplay() failed
        return false;
    }

    if (!eglInitialize(m_egl_display, &egl_major, &egl_minor)) {
        // Error: eglInitialize() failed
        return false;
    }

    // attributes for a visual in RGBA format with at least
    // 8 bits per color
    int attribs[] = {EGL_RENDERABLE_TYPE,
                     EGL_OPENGL_ES3_BIT_KHR,
                     EGL_RED_SIZE,
                     8,
                     EGL_GREEN_SIZE,
                     8,
                     EGL_BLUE_SIZE,
                     8,
                     EGL_SURFACE_TYPE,
                     EGL_WINDOW_BIT,
                     EGL_NONE};

    std::vector<EGLint> ctx_attribs{EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};

    if (!eglChooseConfig(m_egl_display, attribs, &m_config, 1, &num_configs)) {
        // Error: couldn't get an EGL visual config
        return false;
    }

    eglBindAPI(EGL_OPENGL_ES_API);

    if (!m_egl_context) {
        m_egl_context = eglCreateContext(m_egl_display, m_config, EGL_NO_CONTEXT, &ctx_attribs[0]);
        m_attribs = std::move(ctx_attribs);

        if (!m_egl_context) {
            // Error: eglCreateContext failed
            return false;
        }
    }

    if (!CreateWindowSurface()) {
        // Error: CreateWindowSurface failed
        return false;
    }

    MakeCurrent();
    return gladLoadGLES2Loader(EGL_GetFuncAddress);
}

std::unique_ptr<EGLAndroid> EGLAndroid::CreateSharedContext() {
    eglBindAPI(EGL_OPENGL_ES_API);
    EGLContext new_egl_context =
        eglCreateContext(m_egl_display, m_config, m_egl_context, m_attribs.data());
    if (!new_egl_context) {
        // Error: eglCreateContext failed
        return nullptr;
    }

    std::unique_ptr<EGLAndroid> new_context = std::make_unique<EGLAndroid>();
    new_context->m_egl_context = new_egl_context;
    new_context->m_egl_display = m_egl_display;
    new_context->m_config = m_config;
    new_context->m_is_shared = true;
    if (!new_context->CreateWindowSurface()) {
        // Error: CreateWindowSurface failed
        return nullptr;
    }

    return new_context;
}

void EGLAndroid::UpdateSurface(ANativeWindow* surface) {
    m_new_window = surface;
}

void EGLAndroid::UpdateLayout() {
    UpdateCurrentFramebufferLayout(window_width, window_height);
}

bool EGLAndroid::CreateWindowSurface() {
    if (m_host_window) {
        EGLNativeWindowType native_window = GetEGLNativeWindow(m_config);
        m_egl_surface = eglCreateWindowSurface(m_egl_display, m_config, native_window, nullptr);
        if (!m_egl_surface) {
            // Error: eglCreateWindowSurface failed
            return false;
        }
    }
    return true;
}

void EGLAndroid::DestroyWindowSurface() {
    if (m_egl_surface == EGL_NO_SURFACE)
        return;

    if (eglGetCurrentSurface(EGL_DRAW) == m_egl_surface) {
        eglMakeCurrent(m_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }

    if (!eglDestroySurface(m_egl_display, m_egl_surface)) {
        // Could not destroy window surface
    }

    m_egl_surface = EGL_NO_SURFACE;
}

EGLDisplay EGLAndroid::OpenEGLDisplay() {
    return eglGetDisplay(EGL_DEFAULT_DISPLAY);
}

EGLNativeWindowType EGLAndroid::GetEGLNativeWindow(EGLConfig config) {
    EGLint format;
    eglGetConfigAttrib(m_egl_display, config, EGL_NATIVE_VISUAL_ID, &format);
    ANativeWindow_setBuffersGeometry(m_host_window, 0, 0, format);
    window_width = ANativeWindow_getWidth(m_host_window);
    window_height = ANativeWindow_getHeight(m_host_window);
    UpdateCurrentFramebufferLayout(window_width, window_height);
    return static_cast<EGLNativeWindowType>(m_host_window);
}

void EGLAndroid::DestroyContext() {
    if (!m_egl_context)
        return;

    if (eglGetCurrentContext() == m_egl_context) {
        eglMakeCurrent(m_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }
    if (!eglDestroyContext(m_egl_display, m_egl_context)) {
        // Could not destroy drawing context
    }
    if (!m_is_shared && !eglTerminate(m_egl_display)) {
        // Could not destroy display connection
    }
    m_egl_context = EGL_NO_CONTEXT;
    m_egl_display = EGL_NO_DISPLAY;
}

void EGLAndroid::SwapBuffers() {
    eglSwapBuffers(m_egl_display, m_egl_surface);
}

void EGLAndroid::PollEvents() {
    if (m_new_window != nullptr) {
        m_host_window = m_new_window;
        m_new_window = nullptr;
        DoneCurrent();
        DestroyWindowSurface();
        CreateWindowSurface();
        MakeCurrent();
    }
}

void EGLAndroid::MakeCurrent() {
    eglMakeCurrent(m_egl_display, m_egl_surface, m_egl_surface, m_egl_context);
}

void EGLAndroid::DoneCurrent() {
    eglMakeCurrent(m_egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
}
