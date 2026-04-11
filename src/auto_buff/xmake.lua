add_requires("opencv")

add_includedirs("include")
add_includedirs("../common_defs")

target("buff_detector")
    set_kind("binary")
    add_includedirs("../tools")
    add_files("src/main_detector.cpp", "src/buff_detector.cpp",
              "../tools/src/ballistic_models.cpp",
              "../tools/src/camera_params_changer.cpp",
              "../tools/src/logger.cpp",
              "../tools/src/tf_listener.cpp",
              "../common_defs/src/Transform.cpp")
    add_packages("opencv")
    add_packages("eigen")
    add_packages("yaml-cpp")
    add_packages("quill")
    add_packages("nameof")
    add_deps("fast_tf")
    add_rules("iceoryx_deps")
    add_links("atomic")
    on_load(function(target)
        local home = os.getenv("HOME")
        local candidates = {
            {
                include_dir = "/opt/intel/openvino_2024/runtime/include",
                link_dir = "/opt/intel/openvino_2024/runtime/lib/intel64",
            },
            {
                include_dir = "/opt/intel/openvino_2025/runtime/include",
                link_dir = "/opt/intel/openvino_2025/runtime/lib/intel64",
            },
            {
                include_dir = "/usr/include",
                link_dir = "/usr/lib/x86_64-linux-gnu",
            },
            {
                include_dir = "/usr/local/include",
                link_dir = "/usr/local/lib",
            },
        }

        if home and #home > 0 then
            table.insert(candidates, 1, {
                include_dir = path.join(home, "intel/openvino_2024/runtime/include"),
                link_dir = path.join(home, "intel/openvino_2024/runtime/lib/intel64"),
            })
        end

        local found = false
        for _, candidate in ipairs(candidates) do
            if os.isfile(path.join(candidate.include_dir, "openvino/openvino.hpp")) and
               os.isdir(candidate.link_dir) then
                target:add("includedirs", candidate.include_dir)
                target:add("linkdirs", candidate.link_dir)
                target:add("rpathdirs", candidate.link_dir)
                target:add("links", "openvino")
                target:add("defines", "AUTO_BUFF_HAS_OPENVINO=1")
                found = true
                break
            end
        end

        if not found then
            target:add("defines", "AUTO_BUFF_HAS_OPENVINO=0")
        end
    end)
target_end()

target("buff_detector_offline")
    set_kind("binary")
    add_includedirs("../tools")
    add_files("src/main_detector.cpp", "src/buff_detector.cpp",
              "../tools/src/ballistic_models.cpp",
              "../tools/src/camera_params_changer.cpp",
              "../tools/src/logger.cpp",
              "../tools/src/tf_listener.cpp",
              "../common_defs/src/Transform.cpp")
    add_packages("opencv")
    add_packages("eigen")
    add_packages("yaml-cpp")
    add_packages("quill")
    add_packages("nameof")
    add_deps("fast_tf")
    add_rules("iceoryx_deps")
    add_links("atomic")
    add_defines("AUTO_BUFF_FORCE_VIDEO_FILE=1")
    on_load(function(target)
        local home = os.getenv("HOME")
        local candidates = {
            {
                include_dir = "/opt/intel/openvino_2024/runtime/include",
                link_dir = "/opt/intel/openvino_2024/runtime/lib/intel64",
            },
            {
                include_dir = "/opt/intel/openvino_2025/runtime/include",
                link_dir = "/opt/intel/openvino_2025/runtime/lib/intel64",
            },
            {
                include_dir = "/usr/include",
                link_dir = "/usr/lib/x86_64-linux-gnu",
            },
            {
                include_dir = "/usr/local/include",
                link_dir = "/usr/local/lib",
            },
        }

        if home and #home > 0 then
            table.insert(candidates, 1, {
                include_dir = path.join(home, "intel/openvino_2024/runtime/include"),
                link_dir = path.join(home, "intel/openvino_2024/runtime/lib/intel64"),
            })
        end

        local found = false
        for _, candidate in ipairs(candidates) do
            if os.isfile(path.join(candidate.include_dir, "openvino/openvino.hpp")) and
               os.isdir(candidate.link_dir) then
                target:add("includedirs", candidate.include_dir)
                target:add("linkdirs", candidate.link_dir)
                target:add("rpathdirs", candidate.link_dir)
                target:add("links", "openvino")
                target:add("defines", "AUTO_BUFF_HAS_OPENVINO=1")
                found = true
                break
            end
        end

        if not found then
            target:add("defines", "AUTO_BUFF_HAS_OPENVINO=0")
        end
    end)
target_end()

target("buff_tracker")
    set_kind("binary")
    add_includedirs("../tools")
    add_files("src/main_tracker.cpp", "src/tracker_node.cpp", "src/planner.cpp",
              "../tools/src/ballistic_models.cpp",
              "../tools/src/ballistic_trajectory.cpp",
              "../tools/src/logger.cpp",
              "../tools/src/tf_listener.cpp",
              "../common_defs/src/Transform.cpp")
    add_packages("opencv")
    add_packages("eigen")
    add_packages("quill")
    add_packages("nameof")
    add_deps("tinympc")
    add_deps("fast_tf")
    add_rules("iceoryx_deps")
    add_links("atomic")
target_end()

target("buff_video_publisher")
    set_kind("binary")
    add_files("src/video_publisher.cpp")
    add_packages("opencv")
    add_packages("yaml-cpp")
    add_rules("iceoryx_deps")
    add_links("atomic")
target_end()
