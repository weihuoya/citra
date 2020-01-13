// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <unordered_map>
#include <boost/variant.hpp>
#include "video_core/renderer_opengl/gl_shader_manager.h"

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
    explicit OGLShaderStage(bool separable) {
        if (separable) {
            shader_or_program = OGLProgram();
        } else {
            shader_or_program = OGLShader();
        }
    }

    void Create(std::string&& source, GLenum type) {
        shader_type = type;
        shader_code = std::move(source);
        if (shader_or_program.which() == 0) {
            boost::get<OGLShader>(shader_or_program).Create(shader_code.c_str(), type);
        } else {
            OGLShader shader;
            shader.Create(shader_code.c_str(), type);
            OGLProgram& program = boost::get<OGLProgram>(shader_or_program);
            program.Create(true, {shader.handle});
            SetShaderUniformBlockBindings(program.handle);
            SetShaderSamplerBindings(program.handle);
        }
    }

    GLuint GetHandle() const {
        if (shader_or_program.which() == 0) {
            return boost::get<OGLShader>(shader_or_program).handle;
        } else {
            return boost::get<OGLProgram>(shader_or_program).handle;
        }
    }

private:
    friend struct OGLShaderCacheEntity;
    boost::variant<OGLShader, OGLProgram> shader_or_program;
    std::string shader_code;
    GLenum shader_type = 0;
};

class TrivialVertexShader {
public:
    explicit TrivialVertexShader(bool separable) : program(separable) {
        program.Create(GenerateTrivialVertexShader(separable), GL_VERTEX_SHADER);
    }

    GLuint GetHandle() const {
        return program.GetHandle();
    }

private:
    OGLShaderStage program;
};

struct OGLShaderCacheEntity {
    OGLShaderCacheEntity() = default;

    OGLShaderCacheEntity(u64 hash, OGLShaderStage& shader)
        : hashes(1, hash), shader_code(std::move(shader.shader_code)),
          shader_type(shader.shader_type) {}

    std::size_t Save(FileUtil::IOFile& file) {
        std::size_t data_size;
        std::size_t write_size;
        std::size_t total_size = 0;

        // shader
        std::size_t code_size = shader_code.size();
        data_size = sizeof(code_size);
        write_size = file.WriteBytes(&code_size, data_size);
        if (write_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Save failed: code size!");
            return static_cast<std::size_t>(-1);
        }
        total_size += write_size;

        data_size = code_size;
        write_size = file.WriteBytes(shader_code.data(), data_size);
        if (write_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Save failed: shader code!");
            return static_cast<std::size_t>(-1);
        }
        total_size += write_size;

        // type
        data_size = sizeof(shader_type);
        write_size = file.WriteBytes(&shader_type, data_size);
        if (write_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Save failed: shader type!");
            return static_cast<std::size_t>(-1);
        }
        total_size += write_size;

        // hashes
        std::size_t hash_count = hashes.size();
        data_size = sizeof(hash_count);
        write_size = file.WriteBytes(&hash_count, data_size);
        if (write_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Save failed: hash count!");
            return static_cast<std::size_t>(-1);
        }
        total_size += write_size;

        data_size = hash_count * sizeof(u64);
        write_size = file.WriteBytes(hashes.data(), data_size);
        if (write_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Save failed: hashes!");
            return static_cast<std::size_t>(-1);
        }
        total_size += write_size;

        return total_size;
    }

    std::size_t Load(FileUtil::IOFile& file) {
        std::size_t data_size;
        std::size_t read_size;
        std::size_t total_size = 0;

        // shader
        std::size_t code_size = 0;
        data_size = sizeof(code_size);
        read_size = file.ReadBytes(&code_size, data_size);
        if (read_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Load failed: code size!");
            return static_cast<std::size_t>(-1);
        }
        total_size += read_size;

        data_size = code_size;
        shader_code.resize(code_size);
        read_size = file.ReadBytes(shader_code.data(), data_size);
        if (read_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Load failed: shader code!");
            return static_cast<std::size_t>(-1);
        }
        total_size += read_size;

        // type
        data_size = sizeof(shader_type);
        read_size = file.ReadBytes(&shader_type, data_size);
        if (read_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Load failed: shader type!");
            return static_cast<std::size_t>(-1);
        }
        total_size += read_size;

        // hashes
        std::size_t hash_count = 0;
        data_size = sizeof(hash_count);
        read_size = file.ReadBytes(&hash_count, data_size);
        if (read_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Load failed: hash count!");
            return static_cast<std::size_t>(-1);
        }
        total_size += read_size;

        data_size = hash_count * sizeof(u64);
        hashes.resize(hash_count);
        read_size = file.ReadBytes(hashes.data(), data_size);
        if (read_size != data_size) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheEntity Load failed: hashes!");
            return static_cast<std::size_t>(-1);
        }
        total_size += read_size;

        return total_size;
    }

    std::vector<u64> hashes;
    std::string shader_code;
    GLenum shader_type = 0;
};

static std::string OGLShaderCacheFile() {
    u64 program_id = 0;
    Core::System::GetInstance().GetAppLoader().ReadProgramId(program_id);
    const std::string& log_dir = FileUtil::GetUserPath(FileUtil::UserPath::LogDir);
    return fmt::format("{}{:016X}.cache", log_dir, program_id);
}

#define SHADER_CACHE_VERSION 0x1

static void OGLShaderCacheSave(bool separable, std::unordered_map<u64, OGLShaderStage>& shaders,
                               std::unordered_map<u64, OGLShaderStage*>& shaders_ref,
                               std::unordered_map<u64, OGLProgram>& programs) {
    FileUtil::IOFile file;
    if (!file.Open(OGLShaderCacheFile(), "wb")) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheSave file open failed!");
        return;
    }

    std::unordered_map<OGLShaderStage*, OGLShaderCacheEntity> caches;
    for (auto& iter : shaders) {
        caches.emplace(&iter.second, OGLShaderCacheEntity{iter.first, iter.second});
    }

    for (auto& iter_ref : shaders_ref) {
        auto& entity = caches[iter_ref.second];
        entity.hashes.push_back(iter_ref.first);
    }

    std::size_t total_size = 0;

    // version
    std::size_t version = SHADER_CACHE_VERSION;
    std::size_t data_size = sizeof(version);
    std::size_t write_size = file.WriteBytes(&version, data_size);
    if (write_size != data_size) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheSave failed: version!");
        return;
    }
    total_size += write_size;

    // shader count
    std::size_t item_count = caches.size();
    data_size = sizeof(item_count);
    write_size = file.WriteBytes(&item_count, data_size);
    if (write_size != data_size) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheSave failed: shader count!");
        return;
    }
    total_size += write_size;

    // shaders
    for (auto& iter : caches) {
        data_size = iter.second.Save(file);
        if (data_size == std::size_t(-1)) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheSave failed: shader error!");
            return;
        }
        if (!file.IsGood()) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheSave failed: file is bad!");
            return;
        }
        total_size += data_size;
    }
}

static void OGLShaderCacheLoad(bool separable, std::unordered_map<u64, OGLShaderStage>& shaders,
                               std::unordered_map<u64, OGLShaderStage*>& shaders_ref,
                               std::unordered_map<u64, OGLProgram>& programs) {
    FileUtil::IOFile file;
    if (!file.Open(OGLShaderCacheFile(), "rb")) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheLoad file open failed!");
        return;
    }

    std::size_t total_size = file.GetSize();

    // version
    std::size_t version = 0;
    std::size_t data_size = sizeof(version);
    std::size_t read_size = file.ReadBytes(&version, data_size);
    if (read_size != data_size || version != SHADER_CACHE_VERSION) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheLoad failed: version! version: {}", version);
        return;
    }
    total_size -= read_size;

    // shader count
    std::size_t item_count = 0;
    data_size = sizeof(item_count);
    read_size = file.ReadBytes(&item_count, data_size);
    if (read_size != data_size) {
        LOG_DEBUG(Render_OpenGL, "OGLShaderCacheLoad failed: shader count!");
        return;
    }
    total_size -= read_size;

    // shaders
    OGLShaderCacheEntity entity;
    while (item_count > 0 && total_size > 0) {
        data_size = entity.Load(file);
        if (data_size == std::size_t(-1)) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheLoad failed: shader error!");
            return;
        } else {
            auto [iter, new_shader] = shaders.emplace(entity.hashes[0], separable);
            iter->second.Create(std::move(entity.shader_code), entity.shader_type);
            for (std::size_t i = 1; i < entity.hashes.size(); ++i) {
                shaders_ref[entity.hashes[i]] = &iter->second;
            }
        }
        if (!file.IsGood()) {
            LOG_DEBUG(Render_OpenGL, "OGLShaderCacheLoad failed: file is bad!");
            return;
        }

        item_count -= 1;
        total_size -= data_size;
    }
}

class ShaderProgramManager::Impl {
public:
    Impl(bool separable, bool is_amd)
        : separable(separable), is_amd(is_amd), trivial_vertex_shader(separable) {
        if (separable) {
            pipeline.Create();
        }
        // load
        // OGLShaderCacheLoad(separable, shaders, shaders_ref, program_cache);
    }

    ~Impl() {
        // save
        // OGLShaderCacheSave(separable, shaders, shaders_ref, program_cache);
    }

    bool UseProgrammableVertexShader(const Pica::Regs& regs, Pica::Shader::ShaderSetup& setup) {
        PicaVSConfig key(regs, setup);
        u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto iter_ref = shaders_ref.find(key_hash);
        if (iter_ref == shaders_ref.end()) {
            std::string vs_code = GenerateVertexShader(setup, key, separable);
            if (vs_code.empty()) {
                shaders_ref[key_hash] = nullptr;
                current_shader.vs = 0;
            } else {
                u64 code_hash = Common::ComputeHash64(vs_code.data(), vs_code.size());
                auto [iter, new_shader] = shaders.emplace(code_hash, OGLShaderStage{separable});
                OGLShaderStage& cached_shader = iter->second;
                if (new_shader) {
                    cached_shader.Create(std::move(vs_code), GL_VERTEX_SHADER);
                }
                shaders_ref[key_hash] = &cached_shader;
                current_shader.vs = cached_shader.GetHandle();
            }
        } else if (iter_ref->second == nullptr) {
            current_shader.vs = 0;
        } else {
            current_shader.vs = iter_ref->second->GetHandle();
        }
        return (current_shader.vs != 0);
    }

    void UseFixedGeometryShader(const Pica::Regs& regs) {
        PicaFixedGSConfig key(regs);
        u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto [iter, new_shader] = shaders.emplace(key_hash, separable);
        OGLShaderStage& cached_shader = iter->second;
        if (new_shader) {
            std::string gs_code = GenerateFixedGeometryShader(key, separable);
            cached_shader.Create(std::move(gs_code), GL_GEOMETRY_SHADER);
        }
        current_shader.gs = cached_shader.GetHandle();
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
                cached_shader.Create(std::move(fs_code), GL_FRAGMENT_SHADER);
            }
            shaders_ref[key_hash] = &cached_shader;
            current_shader.fs = cached_shader.GetHandle();
        } else {
            current_shader.fs = iter_ref->second->GetHandle();
        }
    }

    void UseTrivialVertexShader() {
        current_shader.vs = trivial_vertex_shader.GetHandle();
    }

    void UseTrivialGeometryShader() {
        current_shader.gs = 0;
    }

    void ApplyTo(OpenGLState& state) {
        if (separable) {
            if (is_amd) {
                // Without this reseting, AMD sometimes freezes when one stage is changed but not
                // for the others. On the other hand, including this reset seems to introduce memory
                // leak in Intel Graphics.
                glUseProgramStages(
                    pipeline.handle,
                    GL_VERTEX_SHADER_BIT | GL_GEOMETRY_SHADER_BIT | GL_FRAGMENT_SHADER_BIT, 0);
            }

            glUseProgramStages(pipeline.handle, GL_VERTEX_SHADER_BIT, current_shader.vs);
            glUseProgramStages(pipeline.handle, GL_GEOMETRY_SHADER_BIT, current_shader.gs);
            glUseProgramStages(pipeline.handle, GL_FRAGMENT_SHADER_BIT, current_shader.fs);
            state.draw.shader_program = 0;
            state.draw.program_pipeline = pipeline.handle;
        } else {
            u64 hash = current_shader.vs; hash <<= 21;
            hash |= current_shader.gs; hash <<= 21;
            hash |= current_shader.fs;
            OGLProgram& cached_program = program_cache[hash];
            if (cached_program.handle == 0) {
                cached_program.Create(false,
                                      {current_shader.vs, current_shader.gs, current_shader.fs});
                SetShaderUniformBlockBindings(cached_program.handle);
                SetShaderSamplerBindings(cached_program.handle);
            }
            state.draw.shader_program = cached_program.handle;
            state.draw.program_pipeline = 0;
        }
    }

private:
    bool separable;
    bool is_amd;

    struct {
        GLuint vs;
        GLuint gs;
        GLuint fs;
    } current_shader;

    TrivialVertexShader trivial_vertex_shader;
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
