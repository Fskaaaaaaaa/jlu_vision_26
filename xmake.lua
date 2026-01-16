set_project("vision")
set_version("0.0.1")
add_rules("mode.debug", "mode.release")
set_languages("c++20")

rule("iceoryx_deps")
on_load(function(target)
	target:add("includedirs", "/usr/local/include/iceoryx/v")
	target:add("linkdirs", "/usr/local/lib")
	target:add("links", "iceoryx_posh", "iceoryx_hoofs", "iceoryx_platform")
end)
rule_end()

rule("openvino_deps")
on_load(function(target)
	target:add("includedirs", "/opt/intel/openvino_2024/runtime/include")
	target:add("linkdirs", "/opt/intel/openvino_2024/runtime/lib/intel64")
	target:add("links", "openvino")
end)
rule_end()

rule("gtsam_deps")
on_load(function(target)
	target:add("includedirs", "/usr/local/include")
	target:add("linkdirs", "/usr/local/lib")
	target:add("links", "gtsam", "metis-gtsam", "cephes-gtsam")
	target:add("rpathdirs", "/usr/local/lib")
	target:add("runenvs", "LD_LIBRARY_PATH", "/usr/local/lib")
	target:add("packages", "eigen")
	target:add("packages", "tbb")
end)
rule_end()

add_requires("opencv")
add_requires("eigen")
add_requires("yaml-cpp", { system = false })
add_requires("reflect-cpp", { configs = { yaml = true } })
--NOTE: 要求cpp20和cmake3.23，正好比ubuntu22官方源高一个版本
--手动下载再用export PATH="/home/aaa/apps/cmake-3.31.10-linux-x86_64/bin:$PATH"指定cmake吧
--另外系统yaml版本也不够反射用，所以得给yaml-cpp的system字段设为false，用高版本的
add_requires("glfw")
add_requires("fmt")
add_requires("tbb")
add_requires("cxxopts")
-- add_requires("magic_enum")
-- add_requires("nameof")
-- add_requires("g2o")
-- FIXME: g2o官方package的deps缺少fmt和spdlog导致链接不通过,
-- 得手动添加到～/.xmake/repositories/xmake-repo/packages/g/g2o/xmake.lua
-- NOTE:这俩反射库也不需要了，直接导reflect-cpp来进行配置加载了
-- NOTE: g2o不需要了，改成导入封装程度更高的gtsam来进行ba优化了

includes("src/third_party")
includes("src/msgs")
includes("src/tools")
includes("src/hardware")
includes("src/coord_tranform")
includes("src/auto_aim")
includes("src/auto_buff")
includes("src/auto_engagement")
includes("src/auto_ballistic")
