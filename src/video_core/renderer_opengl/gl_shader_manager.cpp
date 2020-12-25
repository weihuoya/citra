// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <unordered_map>
#include "core/settings.h"
#include "core/cache_file.h"
#include "video_core/renderer_opengl/gl_shader_manager.h"
#include "video_core/renderer_opengl/on_screen_display.h"

namespace OpenGL {

static void SetShaderUniformBlockBinding(GLuint shader, const char* name, UniformBindings binding,
                                         std::size_t expected_size) {
    const GLuint ub_index = glGetUniformBlockIndex(shader, name);
    if (ub_index == GL_INVALID_INDEX) {
        return;
    }
    GLint ub_size = 0;
    glGetActiveUniformBlockiv(shader, ub_index, GL_UNIFORM_BLOCK_DATA_SIZE, &ub_size);
    ASSERT_MSG(ub_size == expected_size, "Uniform block size did not match! Got {}, expected {}",
               static_cast<int>(ub_size), expected_size);
    glUniformBlockBinding(shader, ub_index, static_cast<GLuint>(binding));
}

static void SetShaderUniformBlockBindings(GLuint shader) {
    SetShaderUniformBlockBinding(shader, "shader_data", UniformBindings::Common,
                                 sizeof(UniformData));
    SetShaderUniformBlockBinding(shader, "vs_config", UniformBindings::VS, sizeof(VSUniformData));
}

static void SetShaderSamplerBinding(GLuint shader, const char* name,
                                    TextureUnits::TextureUnit binding) {
    GLint uniform_tex = glGetUniformLocation(shader, name);
    if (uniform_tex != -1) {
        glUniform1i(uniform_tex, binding.id);
    }
}

static void SetShaderImageBinding(GLuint shader, const char* name, GLuint binding) {
    GLint uniform_tex = glGetUniformLocation(shader, name);
    if (uniform_tex != -1) {
        glUniform1i(uniform_tex, static_cast<GLint>(binding));
    }
}

static void SetShaderSamplerBindings(GLuint shader) {
    GLuint old_program = OpenGLState::BindShaderProgram(shader);

    // Set the texture samplers to correspond to different texture units
    SetShaderSamplerBinding(shader, "tex0", TextureUnits::PicaTexture(0));
    SetShaderSamplerBinding(shader, "tex1", TextureUnits::PicaTexture(1));
    SetShaderSamplerBinding(shader, "tex2", TextureUnits::PicaTexture(2));
    SetShaderSamplerBinding(shader, "tex_cube", TextureUnits::TextureCube);

    // Set the texture samplers to correspond to different lookup table texture units
    SetShaderSamplerBinding(shader, "texture_buffer_lut_light", TextureUnits::TextureBufferLUT_LIGHT);
    SetShaderSamplerBinding(shader, "texture_buffer_lut_fog", TextureUnits::TextureBufferLUT_FOG);
    SetShaderSamplerBinding(shader, "texture_buffer_lut_rg", TextureUnits::TextureBufferLUT_RG);
    SetShaderSamplerBinding(shader, "texture_buffer_lut_rgba", TextureUnits::TextureBufferLUT_RGBA);

    SetShaderImageBinding(shader, "shadow_buffer", ImageUnits::ShadowBuffer);
    SetShaderImageBinding(shader, "shadow_texture_px", ImageUnits::ShadowTexturePX);
    SetShaderImageBinding(shader, "shadow_texture_nx", ImageUnits::ShadowTextureNX);
    SetShaderImageBinding(shader, "shadow_texture_py", ImageUnits::ShadowTexturePY);
    SetShaderImageBinding(shader, "shadow_texture_ny", ImageUnits::ShadowTextureNY);
    SetShaderImageBinding(shader, "shadow_texture_pz", ImageUnits::ShadowTexturePZ);
    SetShaderImageBinding(shader, "shadow_texture_nz", ImageUnits::ShadowTextureNZ);

    OpenGLState::BindShaderProgram(old_program);
}

void PicaUniformsData::SetFromRegs(const Pica::ShaderRegs& regs,
                                   const Pica::Shader::ShaderSetup& setup) {
    std::transform(std::begin(setup.uniforms.b), std::end(setup.uniforms.b), std::begin(bools),
                   [](bool value) -> BoolAligned { return {value ? GL_TRUE : GL_FALSE}; });
    std::transform(std::begin(regs.int_uniforms), std::end(regs.int_uniforms), std::begin(i),
                   [](const auto& value) -> GLuvec4 {
                       return {value.x.Value(), value.y.Value(), value.z.Value(), value.w.Value()};
                   });
    std::transform(std::begin(setup.uniforms.f), std::end(setup.uniforms.f), std::begin(f),
                   [](const auto& value) -> GLvec4 {
                       return {value.x.ToFloat32(), value.y.ToFloat32(), value.z.ToFloat32(),
                               value.w.ToFloat32()};
                   });
}

/**
 * An object representing a shader program staging. It can be either a shader object or a program
 * object, depending on whether separable program is used.
 */
class OGLShaderStage {
public:
    explicit OGLShaderStage(bool separable) : separable(separable) {}

    void Create(const std::string& shader_code, GLenum type, u64 hash) {
        this->hash = hash;
        if (separable) {
            OGLShader shader;
            shader.Create(shader_code.c_str(), type);
            program.Create(true, {shader.handle});
            SetShaderUniformBlockBindings(program.handle);
            if (type == GL_FRAGMENT_SHADER) {
                SetShaderSamplerBindings(program.handle);
            }
        } else {
            this->shader.Create(shader_code.c_str(), type);
        }
    }

    GLuint GetHandle() const {
        if (separable) {
            return program.handle;
        } else {
            return shader.handle;
        }
    }

    u64 GetHash() const {
        return hash;
    }

private:
    OGLShader shader;
    OGLProgram program;
    bool separable;
    u64 hash = 0;
};

class ShaderProgramManager::Impl {
public:
    Impl(bool separable, bool is_amd) : separable(separable), is_amd(is_amd),
            trivial_vertex_shader(separable),
            trivial_geometry_shader(separable) {
        if (separable) {
            pipeline.Create();
        } else if(Settings::values.use_shader_cache) {
            if (LoadProgramCache()) {
                OSD::AddMessage("Shader Cache Loaded!",
                                OSD::MessageType::LOAD_SHADER_CACHE, OSD::Duration::NORMAL, OSD::Color::YELLOW);
            } else {
                OSD::AddMessage("Invalid Shader Cache!",
                                OSD::MessageType::LOAD_SHADER_CACHE, OSD::Duration::NORMAL, OSD::Color::RED);
            }
        }
        trivial_vertex_shader.Create(GenerateTrivialVertexShader(separable), GL_VERTEX_SHADER, 0);
    }

    ~Impl() {
        if (!separable && Settings::values.use_shader_cache) {
            SaveProgramCache();
        }
    }

    bool UseProgrammableVertexShader(const Pica::Regs& regs, Pica::Shader::ShaderSetup& setup) {
        PicaVSConfig key(regs, setup);
        u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto iter_ref = shaders_ref.find(key_hash);
        if (iter_ref == shaders_ref.end()) {
            u64 vkey = key_hash + separable;
            auto viter = vertex_cache.find(vkey);
            if (viter == vertex_cache.end()) {
                vertex_cache[vkey] = GenerateVertexShader(setup, key, separable);
            }
            const std::string& vs_code = vertex_cache[vkey];
            //std::string vs_code = GenerateVertexShader(setup, key, separable);
            if (vs_code.empty()) {
                shaders_ref[key_hash] = nullptr;
                current_shaders.vs = nullptr;
            } else {
                u64 code_hash = Common::ComputeHash64(vs_code.data(), vs_code.size());
                auto [iter, new_shader] = shaders.emplace(code_hash, OGLShaderStage{separable});
                OGLShaderStage& cached_shader = iter->second;
                if (new_shader) {
                    cached_shader.Create(vs_code, GL_VERTEX_SHADER, code_hash);
                    // load cached shader reference
                    auto iter = reference_cache.find(code_hash);
                    if (iter != reference_cache.end()) {
                        for (const auto& hash : iter->second) {
                            shaders_ref[hash] = &cached_shader;
                        }
                    }
                }
                shaders_ref[key_hash] = &cached_shader;
                current_shaders.vs = &cached_shader;
            }
        } else {
            current_shaders.vs = iter_ref->second;
        }
        return (current_shaders.vs != nullptr);
    }

    void UseFixedGeometryShader(const Pica::Regs& regs) {
        PicaFixedGSConfig key(regs);
        u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto [iter, new_shader] = shaders.emplace(key_hash, separable);
        OGLShaderStage& cached_shader = iter->second;
        if (new_shader) {
            std::string gs_code = GenerateFixedGeometryShader(key, separable);
            cached_shader.Create(gs_code, GL_GEOMETRY_SHADER, key_hash);
        }
        current_shaders.gs = &cached_shader;
    }

    void UseFragmentShader(const Pica::Regs& regs) {
        auto key = PicaFSConfig::BuildFromRegs(regs);
        u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto iter_ref = shaders_ref.find(key_hash);
        if (iter_ref == shaders_ref.end()) {
            std::string fs_code = GenerateFragmentShader(key, separable);
            u64 code_hash = Common::ComputeHash64(fs_code.data(), fs_code.size());
            auto [iter, new_shader] = shaders.emplace(code_hash, OGLShaderStage{separable});
            OGLShaderStage& cached_shader = iter->second;
            if (new_shader) {
                cached_shader.Create(fs_code, GL_FRAGMENT_SHADER, code_hash);
                // load cached shader reference
                auto iter = reference_cache.find(code_hash);
                if (iter != reference_cache.end()) {
                    for (const auto& hash : iter->second) {
                        shaders_ref[hash] = &cached_shader;
                    }
                }
            }
            shaders_ref[key_hash] = &cached_shader;
            current_shaders.fs = &cached_shader;
        } else {
            current_shaders.fs = iter_ref->second;
        }
    }

    void UseTrivialVertexShader() {
        current_shaders.vs = &trivial_vertex_shader;
    }

    void UseTrivialGeometryShader() {
        current_shaders.gs = &trivial_geometry_shader;
    }

    void ApplyTo(OpenGLState& state) {
        GLuint vs = current_shaders.vs->GetHandle();
        GLuint gs = current_shaders.gs->GetHandle();
        GLuint fs = current_shaders.fs->GetHandle();

        if (separable) {
            if (is_amd) {
                // Without this reseting, AMD sometimes freezes when one stage is changed but not
                // for the others. On the other hand, including this reset seems to introduce memory
                // leak in Intel Graphics.
                glUseProgramStages(
                    pipeline.handle,
                    GL_VERTEX_SHADER_BIT | GL_GEOMETRY_SHADER_BIT | GL_FRAGMENT_SHADER_BIT, 0);
            }
            glUseProgramStages(pipeline.handle, GL_VERTEX_SHADER_BIT, vs);
            glUseProgramStages(pipeline.handle, GL_GEOMETRY_SHADER_BIT, gs);
            glUseProgramStages(pipeline.handle, GL_FRAGMENT_SHADER_BIT, fs);
            state.draw.shader_program = 0;
            state.draw.program_pipeline = pipeline.handle;
        } else {
            const std::array<u64, 3> shaders{
                    current_shaders.vs->GetHash(),
                    current_shaders.gs->GetHash(),
                    current_shaders.fs->GetHash(),
            };
            u64 hash = Common::ComputeHash64(shaders.data(), shaders.size() * sizeof(u64));
            OGLProgram& cached_program = program_cache[hash];
            if (cached_program.handle == 0) {
                CreateProgram(cached_program, hash, vs, gs, fs);
                SetShaderUniformBlockBindings(cached_program.handle);
                SetShaderSamplerBindings(cached_program.handle);
            }
            state.draw.shader_program = cached_program.handle;
            state.draw.program_pipeline = 0;
        }
    }

    void CreateProgram(OGLProgram& program, u64 hash, GLuint vs, GLuint gs, GLuint fs) {
        auto iter = binary_cache.find(hash);
        // load opengl program binary cache
        if (iter != binary_cache.end()) {
            program.Create(iter->second.format, iter->second.binary);
            if (program.handle == 0) {
                // cache data corrupted
                binary_cache.clear();
            }
        }
        if (program.handle == 0) {
            GLenum format;
            std::vector<GLbyte> binary;
            program.Create(false, {vs, gs, fs});
            program.GetProgramBinary(format, binary);
            if (!binary.empty()) {
                binary_cache.emplace(hash, ProgramCacheEntity{format, std::move(binary)});
            } else {
                LOG_DEBUG(Render_OpenGL, "failed to get program binary!");
            }
        }
    }

    static constexpr u32 PROGRAM_CACHE_VERSION = 0x4;

    static std::string GetCacheFile() {
        u64 program_id = 0;
        Core::System::GetInstance().GetAppLoader().ReadProgramId(program_id);
        const std::string& dir = FileUtil::GetUserPath(FileUtil::UserPath::CacheDir);
        return fmt::format("{}{:016X}.cache", dir, program_id);
    }

    void SaveProgramCache() {
        Core::CacheFile file(GetCacheFile(), Core::CacheFile::MODE_SAVE);

        u32 verion = PROGRAM_CACHE_VERSION;
        file.DoHeader(verion);

        s32 count = static_cast<s32>(binary_cache.size());
        file.Do(count);

        u64 hash;
        u32 length;
        GLenum format;
        std::vector<GLbyte> binary;
        for(auto& pair : binary_cache) {
            hash = pair.first;
            format = pair.second.format;
            length = static_cast<u32>(pair.second.binary.size());
            file.Do(hash);
            file.Do(format);
            file.Do(length);
            file.Do(pair.second.binary);
        }

        if (!file.IsGood()) {
            return;
        }

        file.DoMarker("ShadersRef");
        SaveShadersRef(file);
        if (!file.IsGood()) {
            return;
        }

        file.DoMarker("VertexCache");
        file.Do(vertex_cache);
    }

    bool LoadProgramCache() {
        Core::CacheFile file(GetCacheFile(), Core::CacheFile::MODE_LOAD);

        u32 verion = 0;
        file.DoHeader(verion);
        if (verion != PROGRAM_CACHE_VERSION) {
            FileUtil::Delete(GetCacheFile());
            return false;
        }

        s32 count = 0;
        file.Do(count);

        u64 hash;
        u32 length;
        GLenum format;
        std::vector<GLbyte> binary;
        for (s32 i = 0; i < count; ++i) {
            file.Do(hash);
            file.Do(format);
            file.Do(length);
            file.Do(binary);
            binary_cache.emplace(hash, ProgramCacheEntity{format, std::move(binary)});
        }

        if (!file.IsGood()) {
            binary_cache.clear();
            return false;
        }

        file.DoMarker("ShadersRef");
        LoadShadersRef(file);
        if (!file.IsGood()) {
            reference_cache.clear();
            return false;
        }

        file.DoMarker("VertexCache");
        file.Do(vertex_cache);
        if (!file.IsGood()) {
            vertex_cache.clear();
            return false;
        }

        return true;
    }

    void SaveShadersRef(Core::CacheFile& file) {
        for (const auto& ref : shaders_ref) {
            u64 key_hash = ref.first;
            u64 code_hash = ref.second->GetHash();
            auto iter = reference_cache.find(code_hash);
            if (iter != reference_cache.end()) {
                iter->second.insert(key_hash);
            } else {
                reference_cache[code_hash] = {key_hash};
            }
        }

        u32 count = reference_cache.size();
        file.Do(count);

        for (auto& ref : reference_cache) {
            file.Do(ref.first);
            file.Do(ref.second);
        }
    }

    void LoadShadersRef(Core::CacheFile& file) {
        s32 count = 0;
        file.Do(count);

        for (s32 i = 0; i < count; ++i) {
            u64 code_hash = 0;
            std::unordered_set<u64> hash_set;
            file.Do(code_hash);
            file.Do(hash_set);
            reference_cache[code_hash] = std::move(hash_set);
        }
    }

private:
    bool separable;
    bool is_amd;

    struct {
        OGLShaderStage* vs;
        OGLShaderStage* gs;
        OGLShaderStage* fs;
    } current_shaders;

    struct ProgramCacheEntity {
        explicit ProgramCacheEntity(GLenum format, std::vector<GLbyte>&& binary) : format(format), binary(binary) {}
        GLenum format;
        std::vector<GLbyte> binary;
    };
    std::unordered_map<u64, ProgramCacheEntity> binary_cache;
    std::unordered_map<u64, std::unordered_set<u64>> reference_cache;
    std::unordered_map<u64, std::string> vertex_cache;

    OGLShaderStage trivial_vertex_shader;
    OGLShaderStage trivial_geometry_shader;
    std::unordered_map<u64, OGLShaderStage*> shaders_ref;
    std::unordered_map<u64, OGLShaderStage> shaders;

    OGLPipeline pipeline;
    std::unordered_map<u64, OGLProgram> program_cache;
};

ShaderProgramManager::ShaderProgramManager(bool separable, bool is_amd)
    : impl(std::make_unique<Impl>(separable, is_amd)) {}

ShaderProgramManager::~ShaderProgramManager() = default;

bool ShaderProgramManager::UseProgrammableVertexShader(const Pica::Regs& regs,
                                                       Pica::Shader::ShaderSetup& setup) {
    return impl->UseProgrammableVertexShader(regs, setup);
}

void ShaderProgramManager::UseTrivialVertexShader() {
    impl->UseTrivialVertexShader();
}

void ShaderProgramManager::UseFixedGeometryShader(const Pica::Regs& regs) {
    impl->UseFixedGeometryShader(regs);
}

void ShaderProgramManager::UseTrivialGeometryShader() {
    impl->UseTrivialGeometryShader();
}

void ShaderProgramManager::UseFragmentShader(const Pica::Regs& regs) {
    impl->UseFragmentShader(regs);
}

void ShaderProgramManager::ApplyTo(OpenGLState& state) {
    impl->ApplyTo(state);
}
} // namespace OpenGL
