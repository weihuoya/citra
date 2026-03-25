// Copyright 2018 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <algorithm>
#include <fstream>
#include <unordered_map>
#include "common/file_util.h"
#include "core/cache_file.h"
#include "core/core.h"
#include "core/hle/kernel/process.h"
#include "core/settings.h"
#include "video_core/renderer_opengl/gl_shader_manager.h"
#include "video_core/renderer_opengl/on_screen_display.h"

namespace OpenGL {

namespace {

bool IsPokemonTitle(u64 title_id) {
    switch (title_id) {
    case 0x0004000000055D00: // Pokemon X
    case 0x0004000000055E00: // Pokemon Y
    case 0x000400000011C500: // Pokemon Alpha Sapphire
    case 0x000400000011C400: // Pokemon Omega Ruby
    case 0x00040000001B5000: // Pokemon Ultra Sun
    case 0x00040000001B5100: // Pokemon Ultra Moon
    case 0x0004000000164800: // Pokemon Sun
    case 0x0004000000175E00: // Pokemon Moon
        return true;
    default:
        return false;
    }
}

u64 GetCurrentTitleId() {
    auto& system = Core::System::GetInstance();
    const auto current_process = system.Kernel().GetCurrentProcess();
    if (current_process && current_process->codeset) {
        return current_process->codeset->program_id;
    }

    u64 title_id = 0;
    if (system.GetAppLoader().ReadProgramId(title_id) != Loader::ResultStatus::Success) {
        return 0;
    }
    return title_id;
}

bool ShouldDebugSkipPokemonFragDepthForKey(u64 title_id, u64 key_hash) {
    if (!IsPokemonTitle(title_id)) {
        return false;
    }
    switch (key_hash) {
    case 0xD071D0AD10946707ull:
    case 0x5DCCD2B31FBD5830ull:
    case 0xE918A8FA8474610Full:
    case 0x9786E2FC2AFEC0FDull:
        return true;
    default:
        return false;
    }
}

std::string GetPokemonShaderDumpDir(u64 title_id) {
    return fmt::format("{}pokemon_render/{:016X}/", FileUtil::GetUserPath(FileUtil::UserPath::DumpDir),
                       title_id);
}

std::string BuildVertexSummary(const PicaVSConfig& key) {
    std::string summary = fmt::format(
        "// type=vertex\n// program_hash={:016X}\n// swizzle_hash={:016X}\n// main_offset={}\n"
        "// sanitize_mul={}\n// sanitize_rcp_rsq={}\n// num_outputs={}\n// output_map=",
        key.state.program_hash, key.state.swizzle_hash, key.state.main_offset,
        key.state.sanitize_mul, key.state.sanitize_rcp_rsq ? 1 : 0, key.state.num_outputs);
    for (std::size_t i = 0; i < key.state.output_map.size(); ++i) {
        summary += fmt::format("{}{}", i == 0 ? "" : ",", key.state.output_map[i]);
    }
    summary += '\n';
    return summary;
}

std::string BuildFragmentSummary(const PicaFSConfig& key) {
    const auto& state = key.state;
    return fmt::format(
        "// type=fragment\n// logic_op={}\n// alpha_test_func={}\n// scissor_mode={}\n"
        "// texture0_type={}\n// texture2_use_coord1={}\n// combiner_buffer_input=0x{:02X}\n"
        "// depthmap_enable={}\n// fog_mode={}\n// fog_flip={}\n// lighting_enable={}\n"
        "// lighting_src_num={}\n// lighting_enable_shadow={}\n// proctex_enable={}\n"
        "// shadow_rendering={}\n// shadow_texture_orthographic={}\n",
        static_cast<u32>(state.logic_op), static_cast<u32>(state.alpha_test_func),
        static_cast<u32>(state.scissor_test_mode), static_cast<u32>(state.texture0_type),
        state.texture2_use_coord1 ? 1 : 0, state.combiner_buffer_input,
        static_cast<u32>(state.depthmap_enable), static_cast<u32>(state.fog_mode),
        state.fog_flip ? 1 : 0, state.lighting.enable ? 1 : 0, state.lighting.src_num,
        state.lighting.enable_shadow ? 1 : 0, state.proctex.enable ? 1 : 0,
        state.shadow_rendering ? 1 : 0, state.shadow_texture_orthographic ? 1 : 0);
}

void DumpPokemonShaderSource(const char* stage_name, u64 key_hash, u64 code_hash,
                             const std::string& summary, const std::string& shader_code) {
    const u64 title_id = GetCurrentTitleId();
    if (!IsPokemonTitle(title_id)) {
        return;
    }

    const auto dump_dir = GetPokemonShaderDumpDir(title_id);
    if (!FileUtil::CreateFullPath(dump_dir)) {
        LOG_WARNING(Render_OpenGL, "Pokemon shader dump failed to create {}", dump_dir);
        return;
    }

    const auto filepath =
        fmt::format("{}{}_key_{:016X}_code_{:016X}.glsl.txt", dump_dir, stage_name, key_hash,
                    code_hash);
    if (FileUtil::Exists(filepath)) {
        return;
    }

    std::string contents = fmt::format(
        "// title_id={:016X}\n// key_hash={:016X}\n// code_hash={:016X}\n{}{}\n", title_id,
        key_hash, code_hash, summary, shader_code);
    FileUtil::WriteStringToFile(true, filepath, contents);
    LOG_WARNING(Render_OpenGL, "Pokemon shader dump wrote {}", filepath);
}

void LogPokemonShaderPair(u64 vs_hash, u64 fs_hash) {
    static u64 logged_title_id = 0;
    static std::unordered_set<u64> logged_shader_pairs;

    const u64 title_id = GetCurrentTitleId();
    if (!IsPokemonTitle(title_id) || vs_hash == 0 || fs_hash == 0) {
        return;
    }

    if (logged_title_id != title_id) {
        logged_title_id = title_id;
        logged_shader_pairs.clear();
    }

    const std::array<u64, 2> shader_pair{vs_hash, fs_hash};
    const u64 pair_hash = Common::ComputeHash64(shader_pair.data(), sizeof(shader_pair));
    if (!logged_shader_pairs.emplace(pair_hash).second) {
        return;
    }
    if (logged_shader_pairs.size() > 128) {
        return;
    }

    const auto dump_dir = GetPokemonShaderDumpDir(title_id);
    if (!FileUtil::CreateFullPath(dump_dir)) {
        return;
    }
    const auto filepath = fmt::format("{}shader_pairs.log", dump_dir);
    std::ofstream file(filepath, std::ios::app);
    if (!file.is_open()) {
        return;
    }
    file << fmt::format("vs_code={:016X} fs_code={:016X} pair_hash={:016X}\n", vs_hash, fs_hash,
                        pair_hash);
}

} // namespace

static void SetShaderUniformBlockBinding(GLuint shader, const char* name, UniformBindings binding,
                                         std::size_t expected_size) {
    const GLuint ub_index = glGetUniformBlockIndex(shader, name);
    if (ub_index == GL_INVALID_INDEX) {
        return;
    }
    GLint ub_size = 0;
    glGetActiveUniformBlockiv(shader, ub_index, GL_UNIFORM_BLOCK_DATA_SIZE, &ub_size);
    ASSERT(ub_size == expected_size);
    glUniformBlockBinding(shader, ub_index, static_cast<GLuint>(binding));
}

static void SetShaderUniformBlockBindings(GLuint shader) {
    SetShaderUniformBlockBinding(shader, "shader_data", UniformBindings::Common,
                                 sizeof(UniformData));
    SetShaderUniformBlockBinding(shader, "shader_light_data", UniformBindings::Light,
                                 sizeof(UniformLightData));
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
    SetShaderSamplerBinding(shader, "texture_buffer_lut_lf", TextureUnits::TextureBufferLUT_LF);
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
    explicit Impl(bool separable)
        : separable(separable), trivial_vertex_shader(separable),
          trivial_geometry_shader(separable) {
        if (separable) {
            pipeline.Create();
        } else if (Settings::values.use_shader_cache) {
            u64 size = LoadProgramCache();
            if (size > 0) {
                std::string log{"Load Shader Cache"};
                size >>= 20;
                if (size > 0) {
                    log = fmt::format("{} ({}MB)", log, size);
                }
                OSD::AddMessage(log, OSD::MessageType::ShaderCache, OSD::Duration::NORMAL,
                                OSD::Color::YELLOW);
            }
        }
        trivial_vertex_shader.Create(GenerateTrivialVertexShader(separable), GL_VERTEX_SHADER, 0);
    }

    ~Impl() {
        if (!separable && Settings::values.use_shader_cache) {
            SaveProgramCache();
        }
    }

    OGLShaderStage* GetShaderStageRef(const std::string& shader_code, GLenum shader_type) {
        const u64 code_hash = Common::ComputeHash64(shader_code.data(), shader_code.size());
        auto [iter, new_shader] = shaders.emplace(code_hash, separable);
        OGLShaderStage& cached_shader = iter->second;
        if (new_shader) {
            cached_shader.Create(shader_code, shader_type, code_hash);
            if (cached_shader.GetHandle() == 0) {
                LOG_WARNING(Render_OpenGL, "shader {:04X} create failed!", shader_type);
                shaders.erase(code_hash);
                return nullptr;
            }
            // load cached shader reference
            auto iter = reference_cache.find(code_hash);
            if (iter != reference_cache.end()) {
                for (const auto& hash : iter->second) {
                    shaders_ref[hash] = &cached_shader;
                }
            }
        }
        return &cached_shader;
    }

    bool UseProgrammableVertexShader(const Pica::Regs& regs, Pica::Shader::ShaderSetup& setup) {
        bool result = false;
        const PicaVSConfig key(regs, setup);
        const u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        const auto iter_ref = shaders_ref.find(key_hash);
        if (iter_ref == shaders_ref.end()) {
            std::string vs_code = GenerateVertexShader(setup, key, separable);
            if (vs_code.empty()) {
                LOG_WARNING(Render_OpenGL, "generate programmable vertex shader failed!");
                current_shaders.vs = nullptr;
            } else {
                current_shaders.vs = GetShaderStageRef(vs_code, GL_VERTEX_SHADER);
                if (current_shaders.vs) {
                    DumpPokemonShaderSource("vs", key_hash, current_shaders.vs->GetHash(),
                                            BuildVertexSummary(key), vs_code);
                    shaders_ref[key_hash] = current_shaders.vs;
                    vertex_cache.emplace(current_shaders.vs->GetHash(), std::move(vs_code));
                    result = true;
                }
            }
        } else {
            current_shaders.vs = iter_ref->second;
            result = true;
        }
        return result;
    }

    void UseFixedGeometryShader(const Pica::Regs& regs) {
        const PicaFixedGSConfig key(regs);
        const u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto [iter, new_shader] = shaders.emplace(key_hash, separable);
        OGLShaderStage& cached_shader = iter->second;
        if (new_shader) {
            std::string gs_code = GenerateFixedGeometryShader(key, separable);
            cached_shader.Create(gs_code, GL_GEOMETRY_SHADER, key_hash);
        }
        current_shaders.gs = &cached_shader;
    }

    void UseFragmentShader(const Pica::Regs& regs) {
        const auto key = PicaFSConfig::BuildFromRegs(regs);
        const u64 key_hash = Common::ComputeHash64(&key, sizeof(key));
        auto iter_ref = shaders_ref.find(key_hash);
        if (iter_ref == shaders_ref.end()) {
            std::string fs_code = GenerateFragmentShader(key, separable);
            const u64 title_id = GetCurrentTitleId();
            if (ShouldDebugSkipPokemonFragDepthForKey(title_id, key_hash)) {
                constexpr std::string_view needle = "gl_FragDepth = depth;\n";
                const auto pos = fs_code.find(needle);
                if (pos != std::string::npos) {
                    fs_code.replace(pos, needle.size(), "// Pokemon debug: skip gl_FragDepth\n");
                }
            }
            current_shaders.fs = GetShaderStageRef(fs_code, GL_FRAGMENT_SHADER);
            if (current_shaders.fs) {
                DumpPokemonShaderSource("fs", key_hash, current_shaders.fs->GetHash(),
                                        BuildFragmentSummary(key), fs_code);
                shaders_ref[key_hash] = current_shaders.fs;
                fragment_cache.emplace(current_shaders.fs->GetHash(), std::move(fs_code));
            }
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
        LogPokemonShaderPair(current_shaders.vs ? current_shaders.vs->GetHash() : 0,
                             current_shaders.fs ? current_shaders.fs->GetHash() : 0);

        if (separable) {
            glUseProgramStages(pipeline.handle, GL_VERTEX_SHADER_BIT, vs);
            glUseProgramStages(pipeline.handle, GL_GEOMETRY_SHADER_BIT, gs);
            glUseProgramStages(pipeline.handle, GL_FRAGMENT_SHADER_BIT, fs);
            state.draw.shader_program = 0;
            state.draw.program_pipeline = pipeline.handle;
        } else {
            const std::array<u64, 3> bundle{
                current_shaders.vs->GetHash(),
                current_shaders.gs->GetHash(),
                current_shaders.fs->GetHash(),
            };
            u64 hash = Common::ComputeHash64(bundle.data(), bundle.size() * sizeof(u64));
            auto& cached_program = program_cache[hash];
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

    std::pair<u64, u64> GetCurrentVertexFragmentHashes() const {
        const u64 vs_hash = current_shaders.vs ? current_shaders.vs->GetHash() : 0;
        const u64 fs_hash = current_shaders.fs ? current_shaders.fs->GetHash() : 0;
        return {vs_hash, fs_hash};
    }

    static constexpr u32 PROGRAM_CACHE_VERSION = 0x9;

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
        GLenum format;
        std::vector<GLbyte> binary;
        for (auto& pair : binary_cache) {
            hash = pair.first;
            format = pair.second.format;
            file.Do(hash);
            file.Do(format);
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

        file.DoMarker("FragmentCache");
        file.Do(fragment_cache);
    }

    u64 LoadProgramCache() {
        Core::CacheFile file(GetCacheFile(), Core::CacheFile::MODE_LOAD);

        u32 verion = 0;
        file.DoHeader(verion);
        if (verion != PROGRAM_CACHE_VERSION) {
            FileUtil::Delete(GetCacheFile());
            return 0;
        }

        s32 count = 0;
        file.Do(count);

        u64 hash;
        GLenum format;
        std::vector<GLbyte> binary;
        for (s32 i = 0; i < count; ++i) {
            file.Do(hash);
            file.Do(format);
            file.Do(binary);
            binary_cache.emplace(hash, ProgramCacheEntity{format, std::move(binary)});
        }

        if (!file.IsGood()) {
            binary_cache.clear();
            return 0;
        }

        file.DoMarker("ShadersRef");
        LoadShadersRef(file);
        if (!file.IsGood()) {
            reference_cache.clear();
            return 0;
        }

        file.DoMarker("VertexCache");
        file.Do(vertex_cache);
        if (!file.IsGood()) {
            vertex_cache.clear();
            return 0;
        }

        file.DoMarker("FragmentCache");
        file.Do(fragment_cache);
        if (!file.IsGood()) {
            fragment_cache.clear();
            return 0;
        }

        // load vertex shader cache
        std::vector<u64> error_caches;
        for (const auto& entity : vertex_cache) {
            if (!GetShaderStageRef(entity.second, GL_VERTEX_SHADER)) {
                error_caches.push_back(entity.first);
            }
        }
        for (auto key : error_caches) {
            vertex_cache.erase(key);
        }

        // load fragment shader cache
        error_caches.clear();
        for (const auto& entity : fragment_cache) {
            if (!GetShaderStageRef(entity.second, GL_FRAGMENT_SHADER)) {
                error_caches.push_back(entity.first);
            }
        }
        for (auto key : error_caches) {
            fragment_cache.erase(key);
        }

        return file.GetSize();
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

    struct {
        OGLShaderStage* vs;
        OGLShaderStage* gs;
        OGLShaderStage* fs;
    } current_shaders{};

    struct ProgramCacheEntity {
        explicit ProgramCacheEntity(GLenum format, std::vector<GLbyte>&& binary)
            : format(format), binary(binary) {}
        GLenum format;
        std::vector<GLbyte> binary;
    };
    std::unordered_map<u64, ProgramCacheEntity> binary_cache;
    std::unordered_map<u64, std::unordered_set<u64>> reference_cache;
    std::unordered_map<u64, std::string> vertex_cache;
    std::unordered_map<u64, std::string> fragment_cache;

    OGLShaderStage trivial_vertex_shader;
    OGLShaderStage trivial_geometry_shader;
    std::unordered_map<u64, OGLShaderStage*> shaders_ref;
    std::unordered_map<u64, OGLShaderStage> shaders;

    OGLPipeline pipeline;
    std::unordered_map<u64, OGLProgram> program_cache;
};

ShaderProgramManager::ShaderProgramManager(bool separable)
    : impl(std::make_unique<Impl>(separable)) {}

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

std::pair<u64, u64> ShaderProgramManager::GetCurrentVertexFragmentHashes() const {
    return impl->GetCurrentVertexFragmentHashes();
}

} // namespace OpenGL
