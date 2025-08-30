using DelimitedFiles
using Statistics
using Plots
using StatsPlots

# ---------------------------------------------------
function get_latest_file(dir_path::String)
    files = filter(isfile, readdir(dir_path, join=true))
    sort!(files, by=mtime)
    return isempty(files) ? nothing : files[end]
end

function parse_bias_value(filepath::String)
    content = read(filepath, String)
    for line in split(content, '\n')
        if occursin("bias_value double", line)
            tokens = split(line, '=')
            if length(tokens) == 2
                return parse(Float64, strip(tokens[2]))
            end
        end
    end
    return missing
end

function parse_data_file(filepath::String)
    content = read(filepath, String)
    sections = split(content, "10 runs")

    main_results_lines = filter(line -> !isempty(line) && startswith(line, "0;"), 
                                split(sections[3], '\n'))

    main_costs, main_times = Float64[], Float64[]
    for line in main_results_lines
        values = split(line, ';')
        if length(values) >= 10
            push!(main_costs, parse(Float64, strip(values[2])))
            push!(main_times, parse(Float64, strip(values[9])))
        end
    end

    progress_lines = filter(line -> !isempty(line) && !startswith(line, "progress") && !startswith(line, "100 runs"), 
                            split(sections[4], '\n'))

    progress_costs, progress_iterations, progress_times, run_ids = Float64[], Int[], Float64[], Int[]
    
    for (run_id, line) in enumerate(progress_lines)
        entries = split(line, ';')
        for entry in entries
            if !isempty(strip(entry))
                values = split(entry, ',')
                if length(values) >= 2
                    cost_val = strip(values[1])
                    time_val = strip(values[2])
                    push!(progress_costs, cost_val == "inf" ? Inf : parse(Float64, cost_val))
                    push!(progress_times, parse(Float64, time_val))
                    push!(run_ids, run_id)
                end
            end
        end
        # break
    end

    # @show (progress_times)

    # @show progress_costs

    return main_costs, main_times, progress_costs, progress_iterations, progress_times, run_ids
end

function gather_special_results(base_path::String, env_prefix::String)
    # Look inside the given environment directory
    env_path = joinpath(base_path, env_prefix)
    env_path = env_path * "/"

    all_entries = readdir(env_path)
    println("📂 Contents of $env_path:")
    println(all_entries)


    # Look for folders like 2d_simple_block_00, 2d_simple_block_01, etc.
    env_dirs = filter(entry -> isdir(joinpath(env_path, entry)) && occursin(r"^" * env_prefix * r"_\d\d$", entry), all_entries)
    
    if isempty(env_dirs)
        println("❌ No subfolders found matching pattern $(env_prefix)_xx in $env_path")
    else
        println("✅ Found folders: ", env_dirs)
    end

    all_data = []

    for subdir in sort(env_dirs)
        log_dir = joinpath(env_path, subdir, "log")
        if isdir(log_dir)
            log_file = get_latest_file(log_dir)
            if log_file === nothing
                println("⚠️  No log file found in $log_dir")
                continue
            end

            println("🔍 Parsing $log_file")
            bias = parse_bias_value(log_file)
            println("  bias_value: ", bias)

            if bias !== missing
                _, _, progress_costs, _, progress_times, run_ids = parse_data_file(log_file)

                times_to_first, costs_to_first = Float64[], Float64[]
                
                # for run in unique(run_ids)
                #     indices = findall(id -> id == run, run_ids)
                #     run_costs = progress_costs[indices]
                #     run_times = progress_times[indices]

                #     first_idx = findfirst(isfinite, run_costs)
                #     if first_idx !== nothing
                #         push!(times_to_first, run_times[first_idx])
                #         push!(costs_to_first, run_costs[first_idx])
                #         # println(run_times[first_idx])
                #     end
                # end

                for run in unique(run_ids)
                    indices = findall(id -> id == run, run_ids)
                    run_costs = progress_costs[indices]
                    run_times = progress_times[indices]
                
                    # Find first finite cost that occurs before 10 seconds
                    valid_idx = findfirst(i -> isfinite(run_costs[i]) && run_times[i] < 10.0, 1:length(run_costs))
                    if valid_idx !== nothing
                        push!(times_to_first, run_times[valid_idx])
                        push!(costs_to_first, run_costs[valid_idx])
                    end
                end
                
                
                push!(all_data, (bias=bias, times=times_to_first, costs=costs_to_first))
            else
                println("⚠️  No bias_value found in $log_file")
            end
        else
            println("⚠️  Missing log folder: $log_dir")
        end
    end

    return all_data
end


function gather_special_results_10seconds(base_path::String, env_prefix::String)
    # Look inside the given environment directory
    env_path = joinpath(base_path, env_prefix)
    env_path = env_path * "/"

    all_entries = readdir(env_path)
    println("📂 Contents of $env_path:")
    println(all_entries)


    # Look for folders like 2d_simple_block_00, 2d_simple_block_01, etc.
    env_dirs = filter(entry -> isdir(joinpath(env_path, entry)) && occursin(r"^" * env_prefix * r"_\d\d$", entry), all_entries)
    
    if isempty(env_dirs)
        println("❌ No subfolders found matching pattern $(env_prefix)_xx in $env_path")
    else
        println("✅ Found folders: ", env_dirs)
    end

    all_data = []

    for subdir in sort(env_dirs)
        log_dir = joinpath(env_path, subdir, "log")
        if isdir(log_dir)
            log_file = get_latest_file(log_dir)
            if log_file === nothing
                println("⚠️  No log file found in $log_dir")
                continue
            end

            println("🔍 Parsing $log_file")
            bias = parse_bias_value(log_file)
            println("  bias_value: ", bias)

            if bias !== missing
                _, _, progress_costs, _, progress_times, run_ids = parse_data_file(log_file)

                times_to_first, costs_to_first = Float64[], Float64[]
                

                for run in unique(run_ids)
                    indices = findall(id -> id == run, run_ids)
                    run_costs = progress_costs[indices]
                    run_times = progress_times[indices]

                    first_idx = findfirst(isfinite, run_costs)
                    if first_idx !== nothing
                        push!(times_to_first, run_times[first_idx])
                        push!(costs_to_first, run_costs[first_idx])
                        # println(run_times[first_idx])
                    end
                end
                
                push!(all_data, (bias=bias, times=times_to_first, costs=costs_to_first))
            else
                println("⚠️  No bias_value found in $log_file")
            end
        else
            println("⚠️  Missing log folder: $log_dir")
        end
    end

    return all_data
end


function gather_final_results(base_path::String, env_prefix::String)
    # Look inside the given environment directory
    env_path = joinpath(base_path, env_prefix)
    env_path = env_path * "/"

    all_entries = readdir(env_path)
    println("📂 Contents of $env_path:")
    println(all_entries)


    # Look for folders like 2d_simple_block_00, 2d_simple_block_01, etc.
    env_dirs = filter(entry -> isdir(joinpath(env_path, entry)) && occursin(r"^" * env_prefix * r"_\d\d$", entry), all_entries)
    
    if isempty(env_dirs)
        println("❌ No subfolders found matching pattern $(env_prefix)_xx in $env_path")
    else
        println("✅ Found folders: ", env_dirs)
    end

    all_data = []

    for subdir in sort(env_dirs)
        log_dir = joinpath(env_path, subdir, "log")
        if isdir(log_dir)
            log_file = get_latest_file(log_dir)
            if log_file === nothing
                println("⚠️  No log file found in $log_dir")
                continue
            end

            println("🔍 Parsing $log_file")
            bias = parse_bias_value(log_file)
            println("  bias_value: ", bias)

            if bias !== missing
                main_costs, main_times, _, _, _, _ = parse_data_file(log_file)
            
                # Filter out invalid entries (e.g., Inf or missing)
                valid_indices = findall(!isnan, main_costs)
                final_costs = main_costs[valid_indices]
                final_times = main_times[valid_indices]
            
                push!(all_data, (bias=bias, times=final_times, costs=final_costs))
            else
                println("⚠️  No bias_value found in $log_file")
            end
        else
            println("⚠️  Missing log folder: $log_dir")
        end
    end

    return all_data
end


function plot_bias_comparison(all_data, environment_name)
    sorted_data = sort(all_data, by = x -> x.bias)

    # One label per bias
    labels = [string(round(d.bias, digits=3)) for d in sorted_data]

    # One vector of times/costs per bias, each vector has one entry per run (first solution of that run)
    all_times_to_first = [d.times for d in sorted_data]  # list of vectors
    all_costs_to_first = [d.costs for d in sorted_data]

    # @show all_times_to_first[1]
    # @show all_times_to_first[2]
    # @show labels

    # @show length(all_times_to_first[1])

    # Boxplot of time to first solution per run
    p_time = boxplot(
        # labels,
        all_times_to_first,
        title="Time to 10 Sec Solution per Run - $environment_name",
        ylabel="Time (s)",
        xlabel="Bias Value",
        legend=false,
        size=(900, 500),
        xrotation=45,
        fillalpha=0.6,
        linewidth=1.5,
        outliers=true
    )

    # Boxplot of cost of first solution per run
    p_cost = boxplot(
        all_costs_to_first,
        title="Cost of 10 sec Solution per Run - $environment_name",
        ylabel="Cost",
        xlabel="Bias Value",
        legend=false,
        size=(900, 500),
        xrotation=45,
        fillalpha=0.6,
        linewidth=1.5,
        outliers=true
    )

    # Combine both
    p_combined = plot(p_time, p_cost, layout=(1,2), size=(1600, 600))
    savefig(p_combined, "10sec_bias_comparison_$environment_name.png")
    return p_combined
end



function analyze_bias_effects(environment_prefix::String)
    base_path = "results/special/varyK/"
    println("📊 Analyzing bias effects for: $environment_prefix")
    data = gather_special_results(base_path, environment_prefix)
    if isempty(data)
        println("❌ No valid data parsed.")
    else
        plot_bias_comparison(data, environment_prefix)
    end
end


# Example usage:
# include("analyze_bias_comparison.jl")
analyze_bias_effects("2d_new_narrow")

# analyze_bias_effects("2d_simple_block")

# analyze_bias_effects("2d_simple_underwater")

# analyze_bias_effects("2d_unicycle_narrow")

# analyze_bias_effects("2d_unicycle_block")

# analyze_bias_effects("2d_unicycle_underwater")
# 