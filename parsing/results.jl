using DelimitedFiles
using Statistics
using Plots
using StatsPlots

function plot_main_results(main_costs, main_times)
    # Create box plots for main results
    p1 = boxplot(["Cost"], main_costs, 
                title="Main Results: Cost Distribution",
                ylabel="Cost", 
                legend=false,
                linewidth=1.5,
                fillalpha=0.7,
                color=:blue)
    
    p2 = boxplot(["Time"], main_times, 
                title="Main Results: Time Distribution",
                ylabel="Time (seconds)", 
                legend=false,
                linewidth=1.5,
                fillalpha=0.7,
                color=:green)
    
    # Combine plots
    p_main = plot(p1, p2, layout=(1,2), size=(800, 400))
    
    # Print summary statistics
    println("Main Results Statistics:")
    println("  Costs - Mean: $(mean(main_costs)), Median: $(median(main_costs))")
    println("         - Min: $(minimum(main_costs)), Max: $(maximum(main_costs))")
    println("         - Standard Deviation: $(std(main_costs))")
    println("  Times - Mean: $(mean(main_times)), Median: $(median(main_times))")
    println("         - Min: $(minimum(main_times)), Max: $(maximum(main_times))")
    println("         - Standard Deviation: $(std(main_times))")
    
    # Save the plot
    savefig(p_main, "main_results_boxplots.png")
    
    return p_main
end

function create_time_bucketed_boxplots(progress_costs, progress_iterations, progress_times)
    # Define time buckets: 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0, ...
    # Start with small increments and scale up
    # time_buckets = [0.1, 0.2, 0.5, 1.0, 2.5, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 62]
    time_buckets = [0.1, 0.2, 0.5, 1.0, 2.5, 5.0, 10.0, 20, 30, 40, 50, 60]
    # Filter out buckets that exceed our maximum time
    max_time = maximum(progress_times)
    time_buckets = filter(bucket -> bucket <= max_time * 1.1, time_buckets)
    
    # Add the maximum time to ensure we include all data points
    if !isempty(time_buckets) && time_buckets[end] < max_time
        push!(time_buckets, ceil(max_time * 10) / 10)  # Round up to next 0.1
    elseif isempty(time_buckets)
        push!(time_buckets, ceil(max_time * 10) / 10)  # Ensure we have at least one bucket
    end
    
    # Create bucket labels
    bucket_labels = String[]
    for i in 1:length(time_buckets)
        if i == 1
            push!(bucket_labels, "≤$(time_buckets[i])")
        else
            push!(bucket_labels, "$(time_buckets[i-1])-$(time_buckets[i])")
        end
    end
    
    # Initialize data arrays for each bucket
    bucketed_costs = [Float64[] for _ in 1:length(bucket_labels)]
    bucketed_iterations = [Int[] for _ in 1:length(bucket_labels)]
    
    # Assign data to buckets
    for i in 1:length(progress_times)
        time = progress_times[i]
        cost = progress_costs[i]
        # iteration = progress_iterations[i]
        
        # Find the appropriate bucket
        bucket_idx = findfirst(b -> time <= b, time_buckets)
        if !isnothing(bucket_idx)
            # @show iteration
            push!(bucketed_costs[bucket_idx], cost)
            # push!(bucketed_iterations[bucket_idx], iteration)
        end
    end
    
    # Filter out infinite costs for plotting
    finite_bucketed_costs = [filter(isfinite, costs) for costs in bucketed_costs]
    
    # @show finite_bucketed_costs

    # Create box plots

    # @show bucket_labels
    # @show bucketed_costs[1]

    for i in 1:length(finite_bucketed_costs)
        if isempty(finite_bucketed_costs[i])
            finite_bucketed_costs[i] = [0.0]
        end
    end

    # return bucket_labels, finite_bucketed_costs, bucketed_iterations
    x = 1:12

    p_costs = boxplot(x, finite_bucketed_costs, 
             title="Progress Costs vs Time Buckets",
             xlabel="Time Buckets (seconds)",
             ylabel="Cost", 
             legend=false,
             linewidth=1.5,
             fillalpha=0.7,
             color=:blue,
             outliers=true,
             whisker_width=0.5,
             size=(800, 500),
             xrotation=45)
    
    # p_iterations = boxplot(bucket_labels, bucketed_iterations, 
    #              title="Progress Iterations vs Time Buckets",
    #              xlabel="Time Buckets (seconds)",
    #              ylabel="Iterations", 
    #              legend=false,
    #              linewidth=1.5,
    #              fillalpha=0.7,
    #              color=:green,
    #              outliers=true,
    #              whisker_width=0.5,
    #              size=(800, 500),
    #              xrotation=45)
    
    # Print bucket statistics
    println("Time Bucket Statistics:")
    for (i, label) in enumerate(bucket_labels)
        costs = finite_bucketed_costs[i]
        # iterations = bucketed_iterations[i]
        
        println("\nBucket $label ($(length(costs)) data points):")
        
        if !isempty(costs)
            println("  Costs - Mean: $(mean(costs)), Median: $(median(costs))")
            println("         - Min: $(minimum(costs)), Max: $(maximum(costs))")
            println("         - Standard Deviation: $(std(costs))")
        else
            println("  Costs - No finite cost values in this bucket")
        end
        
        # if !isempty(iterations)
        #     println("  Iterations - Mean: $(mean(iterations)), Median: $(median(iterations))")
        #     println("             - Min: $(minimum(iterations)), Max: $(maximum(iterations))")
        #     println("             - Standard Deviation: $(std(iterations))")
        # else
        #     println("  Iterations - No values in this bucket")
        # end
    end
    
    # # Save the plots
    savefig(p_costs, "costs_by_time_buckets.png")

    plot(p_costs)
    # savefig(p_iterations, "iterations_by_time_buckets.png")
    
    # # Combine plots
    # p_combined = plot(p_costs, p_iterations, layout=(2,1), size=(800, 800))
    # savefig(p_combined, "progress_metrics_by_time_buckets.png")
    
    # return p_costs, p_iterations, p_combined, bucket_labels, bucketed_costs, bucketed_iterations
end

using DelimitedFiles
using Statistics
using Plots
using StatsPlots

function parse_data_file(filepath)
    # Read the entire file content
    content = read(filepath, String)
    
    # Split the content based on "100 runs" marker to separate different sections
    sections = split(content, "100 runs")
    println("Number of sections: ", length(sections))
    # @show sections

    # First section: Main results data
    main_results_lines = filter(line -> !isempty(line) && startswith(line, "0;"), 
                                split(sections[3], '\n'))
    
    # Extract cost and time from main results
    main_costs = Float64[]
    main_times = Float64[]
    
    for line in main_results_lines
        values = split(line, ';')
        if length(values) >= 10
            push!(main_costs, parse(Float64, strip(values[2])))
            push!(main_times, parse(Float64, strip(values[9])))
        end
    end
    
    # Second section: Progress properties
    progress_lines = filter(line -> !isempty(line) && !startswith(line, "progress") && !startswith(line, "100 runs"), 
                            split(sections[4], '\n'))
    
    # Extract best cost, iterations, and time from progress properties
    progress_costs = Float64[]
    progress_iterations = Int[]
    progress_times = Float64[]
    run_ids = Int[]
    
    for (run_id, line) in enumerate(progress_lines)
        # @show line
        # break
        # Format is like: inf,381,0.0500599,;inf,440,0.10014,;...
        entries = split(line, ';')
        for entry in entries
            if !isempty(strip(entry))
                values = split(entry, ',')
                if length(values) >= 4
                    # Handle "inf" values
                    cost_val = strip(values[1])
                    if cost_val == "inf"
                        push!(progress_costs, Inf)
                    else
                        push!(progress_costs, parse(Float64, cost_val))
                    end
                    push!(progress_iterations, parse(Int, strip(values[2])))
                    push!(progress_times, parse(Float64, strip(values[3])))
                    push!(run_ids, run_id) 
                elseif length(values) >= 3
                    # Handle "inf" values
                    cost_val = strip(values[1])
                    if cost_val == "inf"
                        push!(progress_costs, Inf)
                    else
                        push!(progress_costs, parse(Float64, cost_val))
                    end
                    
                    push!(progress_times, parse(Float64, strip(values[2])))
                    push!(run_ids, run_id) 
                end
            end
        end
    end
    
    return main_costs, main_times, progress_costs, progress_iterations, progress_times, run_ids
end

function get_latest_file(dir_path)
    # Get all files in the directory
    files = filter(isfile, readdir(dir_path, join=true))
    
    # Sort files by modification time (most recent last)
    sort!(files, by=mtime)
    
    # Return the most recent file (last in the sorted array)
    return isempty(files) ? nothing : files[end]
end

function analyze_first_solutions(environment_name, methods_info)
    """
    Analyze and visualize the time to first solution for different methods.
    
    Arguments:
    - environment_name: String name of the environment (e.g., "2d_simple_narrow")
    - methods_info: Dictionary mapping method names to their respective log file paths
    """
    # Colors for different methods
    method_colors = Dict(
        "fixedK" => :blue,
        "varyK" => :red,
        "RRBT" => :green
    )
    
    # Initialize data containers
    method_times_to_first = Dict()
    method_first_solution_costs = Dict()
    
    # Process data for each method
    for (method_name, filepath) in methods_info
        if isfile(filepath)
            # Parse the data file
            _, _, progress_costs, _, progress_times, run_ids = parse_data_file(filepath)
            
            # Initialize arrays for this method
            times_to_first = Float64[]
            first_solution_costs = Float64[]
            
            # Find unique run IDs
            unique_runs = unique(run_ids)
            
            # For each run, find the first finite cost solution
            for run in unique_runs
                # Get data for this run
                run_indices = findall(id -> id == run, run_ids)
                
                if !isempty(run_indices)
                    run_costs = progress_costs[run_indices]
                    run_times = progress_times[run_indices]
                    
                    # Find the first finite cost
                    first_finite_idx = findfirst(isfinite, run_costs)
                    
                    if !isnothing(first_finite_idx)
                        push!(times_to_first, run_times[first_finite_idx])
                        push!(first_solution_costs, run_costs[first_finite_idx])
                    end
                end
            end
            
            # Store results for this method
            method_times_to_first[method_name] = times_to_first
            method_first_solution_costs[method_name] = first_solution_costs
            
            println("Processed $method_name with $(length(times_to_first)) detected runs")
            if !isempty(times_to_first)
                println("  Time to first solution - Mean: $(mean(times_to_first)), Median: $(median(times_to_first))")
                println("                         - Min: $(minimum(times_to_first)), Max: $(maximum(times_to_first))")
                println("  First solution cost   - Mean: $(mean(first_solution_costs)), Median: $(median(first_solution_costs))")
            else
                println("  No valid first solutions detected")
            end
        else
            println("Warning: File $filepath does not exist")
        end
    end
    
    # Create boxplots for time to first solution
    p_times = boxplot(
        title="Time to First Solution - $environment_name",
        ylabel="Time (seconds)",
        legend=false,
        size=(800, 500),
        xrotation=30,
        margin=10Plots.mm
    )
    
    # Add boxplots for each method
    for (method_name, times) in method_times_to_first
        if !isempty(times)
            boxplot!(
                p_times,
                [method_name],
                [times],
                linewidth=1.5,
                fillalpha=0.7,
                color=method_colors[method_name],
                outliers=true,
                whisker_width=0.5,
                width=0.5
            )
        end
    end
    
    # Create boxplots for first solution costs
    p_costs = boxplot(
        title="First Solution Cost - $environment_name",
        ylabel="Cost",
        legend=false,
        size=(800, 500),
        xrotation=30,
        margin=10Plots.mm
    )
    
    # Add boxplots for each method
    for (method_name, costs) in method_first_solution_costs
        if !isempty(costs)
            boxplot!(
                p_costs,
                [method_name],
                [costs],
                linewidth=1.5,
                fillalpha=0.7,
                color=method_colors[method_name],
                outliers=true,
                whisker_width=0.5,
                width=0.5
            )
        end
    end
    
    # Create a combined plot
    p_combined = plot(p_times, p_costs, layout=(1,2), size=(1600, 600))
    
    # Save the plots
    savefig(p_times, "$(environment_name)_time_to_first_solution.png")
    savefig(p_costs, "$(environment_name)_first_solution_cost.png")
    savefig(p_combined, "$(environment_name)_first_solution_combined.png")
    
    return p_combined, method_times_to_first, method_first_solution_costs
end

function compare_all_environments_first_solution(environments, methods, base_path)
    """
    Create a comparison of time to first solution across all environments.
    
    Arguments:
    - environments: List of environment names
    - methods: Dictionary of method names and their folder names
    - base_path: Base path for result files
    """
    # Initialize data for all plots
    env_plots = Dict()
    
    for env in environments
        println("\nProcessing environment: $env")
        
        # Complete path information for this environment
        env_methods = Dict()
        for (method_key, method_path) in methods
            if method_key == "RRBT"
                env_methods[method_key] = get_latest_file("$base_path/rrbt/$env/log/")
            elseif env == "2d_simple_narrow" && (method_key == "fixedK" || method_key == "varyK")
                env_methods[method_key] = "$base_path/$method_path/$env/log/ompl_Vivaldi_2025-03-09 03:46:17.log"
            else
                env_methods[method_key] = "$base_path/$method_path/$env/log/ompl_Vivaldi_2025-03-09 05:28:36.log"
            end
        end
        
        # Create plots for this environment
        p_combined, _, _ = analyze_first_solutions(env, env_methods)
        env_plots[env] = p_combined
    end
    
    # Create individual comparison plots for each metric (time and cost) across environments
    method_colors = Dict(
        "fixedK" => :blue,
        "varyK" => :red,
        "RRBT" => :green
    )
    
    # Time to first solution across environments
    p_all_times = plot(
        title="Time to First Solution - All Environments",
        ylabel="Time (seconds)",
        legend=:topright,
        size=(1000, 600),
        margin=10Plots.mm
    )
    
    # First solution cost across environments
    p_all_costs = plot(
        title="First Solution Cost - All Environments",
        ylabel="Cost",
        legend=:topright,
        size=(1000, 600),
        margin=10Plots.mm
    )
    
    # Process each environment and method
    for (env_idx, env) in enumerate(environments)
        # Complete path information for this environment
        env_methods = Dict()
        for (method_key, method_path) in methods
            if method_key == "RRBT"
                env_methods[method_key] = get_latest_file("$base_path/rrbt/$env/log/")
            elseif env == "2d_simple_narrow" && (method_key == "fixedK" || method_key == "varyK")
                env_methods[method_key] = "$base_path/$method_path/$env/log/ompl_Vivaldi_2025-03-09 03:46:17.log"
            else
                env_methods[method_key] = "$base_path/$method_path/$env/log/ompl_Vivaldi_2025-03-09 05:28:36.log"
            end
        end
        
        # Process each method
        for (method_idx, (method_name, filepath)) in enumerate(env_methods)
            if isfile(filepath)
                # Parse the data file
                _, _, progress_costs, _, progress_times, run_ids = parse_data_file(filepath)
                
                # Find unique run IDs
                unique_runs = unique(run_ids)
                
                times_to_first = Float64[]
                first_solution_costs = Float64[]
                
                # For each run, find the first finite cost solution
                for run in unique_runs
                    # Get data for this run
                    run_indices = findall(id -> id == run, run_ids)
                    
                    if !isempty(run_indices)
                        run_costs = progress_costs[run_indices]
                        run_times = progress_times[run_indices]
                        
                        # Find the first finite cost
                        first_finite_idx = findfirst(isfinite, run_costs)
                        
                        if !isnothing(first_finite_idx)
                            push!(times_to_first, run_times[first_finite_idx])
                            push!(first_solution_costs, run_costs[first_finite_idx])
                        end
                    end
                end
                
                # Add to the comparison plots
                if !isempty(times_to_first)
                    # Position offset for methods within an environment
                    offset = (method_idx - 1) * 0.25 - 0.25
                    
                    # Add to time comparison
                    boxplot!(
                        p_all_times,
                        [env_idx + offset],
                        [times_to_first],
                        label=(env_idx == 1 ? method_name : nothing),  # Only label once
                        linewidth=1.5,
                        fillalpha=0.7,
                        color=method_colors[method_name],
                        outliers=true,
                        whisker_width=0.12,
                        width=0.2
                    )
                    
                    # Add to cost comparison
                    boxplot!(
                        p_all_costs,
                        [env_idx + offset],
                        [first_solution_costs],
                        label=(env_idx == 1 ? method_name : nothing),  # Only label once
                        linewidth=1.5,
                        fillalpha=0.7,
                        color=method_colors[method_name],
                        outliers=true,
                        whisker_width=0.12,
                        width=0.2
                    )
                end
            end
        end
    end
    
    # Set x-axis labels to environment names
    xticks!(p_all_times, 1:length(environments), environments)
    xticks!(p_all_costs, 1:length(environments), environments)
    
    # Save the combined plots
    savefig(p_all_times, "all_environments_time_to_first_solution.png")
    savefig(p_all_costs, "all_environments_first_solution_cost.png")
    
    # Create a final combined plot with both metrics
    p_all_combined = plot(p_all_times, p_all_costs, layout=(2,1), size=(1000, 1000))
    savefig(p_all_combined, "all_environments_first_solution_combined.png")
    
    return p_all_combined, p_all_times, p_all_costs
end



function analyze_data_file(filepath)
    # Parse the data file
    main_costs, main_times, progress_costs, progress_iterations, progress_times = parse_data_file(filepath)
    
    # Create separate plots
    p_main = plot_main_results(main_costs, main_times)
    # p_progress = plot_progress_results(progress_costs, progress_iterations, progress_times)
    
    return main_costs, main_times, progress_costs, progress_iterations, progress_times, p_main
end


function analyze_progress_by_time_buckets(filepath)
    # Parse the data file
    main_costs, main_times, progress_costs, progress_iterations, progress_times = parse_data_file(filepath)
    
    plot_main_results(main_costs, main_times)

    # Create time-bucketed box plots
    # p_costs, p_iterations, p_combined, bucket_labels, bucketed_costs, bucketed_iterations = 
        # create_time_bucketed_boxplots(progress_costs, progress_iterations, progress_times)
    
    create_time_bucketed_boxplots(progress_costs, progress_iterations, progress_times)

    # return labels, costs, iterations

    # return p_costs, p_iterations, p_combined, main_costs, main_times, 
        #    progress_costs, progress_iterations, progress_times, 
        #    bucket_labels, bucketed_costs, bucketed_iterations
end

# analyze_progress_by_time_buckets("results/final/rrbt/2d_simple_block/log/ompl_Vivaldi_2024-07-07 03:05:49.log")

# analyze_progress_by_time_buckets("results/final/fixedK/2d_simple_block/log/ompl_Vivaldi_2024-07-07 03:03:12.log")

# analyze_progress_by_time_buckets("results/final/varyK/2d_simple_block/log/ompl_Vivaldi_2024-07-07 03:03:12.log")

# Narrow passageway

# analyze_progress_by_time_buckets("results/final/fixedK/2d_simple_narrow/log/ompl_Vivaldi_2024-07-07 04:44:24.log")

# analyze_progress_by_time_buckets("results/final/rrbt/2d_simple_narrow/log/ompl_Vivaldi_2024-07-06 20:31:24.log")

# analyze_progress_by_time_buckets("results/final/varyK/2d_simple_narrow/log/ompl_Vivaldi_2024-07-07 04:44:26.log")

function get_latest_file(dir_path)
    # Get all files in the directory
    files = filter(isfile, readdir(dir_path, join=true))
    
    # Sort files by modification time (most recent last)
    sort!(files, by=mtime)
    
    # Return the most recent file (last in the sorted array)
    return isempty(files) ? nothing : files[end]
end

function compare_methods_by_time_buckets(environment_name, methods_info, time_buckets=[0.1, 0.5, 1.0, 5.0, 10.0, 20.0])
    """
    Create boxplots comparing different methods based on time buckets.
    
    Arguments:
    - environment_name: String name of the environment (e.g., "2d_simple_narrow")
    - methods_info: Dictionary mapping method names to their respective log file paths
    - time_buckets: Array of time thresholds to create buckets
    """
    # Colors for different methods
    method_colors = Dict(
        "fixedK" => :blue,
        "varyK" => :red,
        "RRBT" => :green
    )
    
    # Create bucket labels
    bucket_labels = String[]
    for i in 1:length(time_buckets)
        if i == 1
            push!(bucket_labels, "≤$(time_buckets[i])")
        else
            push!(bucket_labels, "$(time_buckets[i-1])-$(time_buckets[i])")
        end
    end
    
    # Initialize data structure to hold costs per method per time bucket
    method_bucketed_costs = Dict()
    
    # Process data for each method
    for (method_name, filepath) in methods_info
        if isfile(filepath)
            # Parse the data file
            _, _, progress_costs, _, progress_times = parse_data_file(filepath)
            
            # Initialize buckets for this method
            method_bucketed_costs[method_name] = [Float64[] for _ in 1:length(bucket_labels)]
            
            # Assign costs to time buckets
            for i in 1:length(progress_times)
                time = progress_times[i]
                cost = progress_costs[i]
                
                # Find the appropriate bucket
                bucket_idx = findfirst(b -> time <= b, time_buckets)
                if !isnothing(bucket_idx)
                    push!(method_bucketed_costs[method_name][bucket_idx], cost)
                end
            end
            
            # Filter out infinite costs
            for i in 1:length(method_bucketed_costs[method_name])
                method_bucketed_costs[method_name][i] = filter(isfinite, method_bucketed_costs[method_name][i])
                
                # Fill empty buckets with a placeholder value to avoid NaN errors
                if isempty(method_bucketed_costs[method_name][i])
                    # Use a placeholder value - we'll skip plotting these later
                    method_bucketed_costs[method_name][i] = []
                end
            end
            
            println("Processed $method_name with progress data points")
        else
            println("Warning: File $filepath does not exist")
        end
    end
    
    # Create the comparison plot
    p = plot(
        title="Cost Progression by Time - $environment_name",
        xlabel="Time Buckets (seconds)",
        ylabel="Cost",
        size=(900, 600),
        legend=:topright,
        xrotation=45,
        margin=10Plots.mm
    )
    
    # Box offset values to avoid overlap
    num_methods = length(methods_info)
    offsets = LinRange(-0.2, 0.2, num_methods)
    
    # Add boxplots for each method
    for (i, (method_name, _)) in enumerate(methods_info)
        if haskey(method_bucketed_costs, method_name)
            # For each time bucket, check if we have valid data
            for j in 1:length(bucket_labels)
                # Skip empty buckets or buckets with just placeholder values
                if !isempty(method_bucketed_costs[method_name][j])
                    boxplot!(
                        p,
                        [j + offsets[i]],  # Single position for this boxplot
                        [method_bucketed_costs[method_name][j]],  # Data for this bucket
                        label=(j == 1 ? method_name : nothing),  # Only label once
                        linewidth=1.5,
                        fillalpha=0.7,
                        color=method_colors[method_name],
                        outliers=true,
                        whisker_width=0.5,
                        width=0.3
                    )
                end
            end
        end
    end
    
    # Set x-axis labels to bucket labels
    xticks!(p, 1:length(bucket_labels), bucket_labels)
    
    # Print summary statistics
    println("\nSummary Statistics by Method and Time Bucket - $environment_name:")
    for (method_name, buckets) in method_bucketed_costs
        println("\n$method_name:")
        for (i, bucket) in enumerate(buckets)
            costs = filter(isfinite, bucket)
            if !isempty(costs)
                println("  Bucket $(bucket_labels[i]) ($(length(costs)) points):")
                println("    Mean: $(mean(costs)), Median: $(median(costs))")
                println("    Min: $(minimum(costs)), Max: $(maximum(costs))")
                if length(costs) > 1
                    println("    Standard Deviation: $(std(costs))")
                end
            else
                println("  Bucket $(bucket_labels[i]): No valid cost data")
            end
        end
    end
    
    # Save plot
    savefig(p, "$(environment_name)_time_buckets_comparison.png")
    
    return p, method_bucketed_costs, bucket_labels
end

# Example usage
latest_file_rrbt = get_latest_file("results/final/rrbt/2d_simple_underwater/log/")
# println("Latest file: ", latest_file)

# Underwater

analyze_progress_by_time_buckets("results/final/fixedK/2d_simple_underwater/log/ompl_Vivaldi_2025-03-09 05:28:36.log")

analyze_progress_by_time_buckets("/home/qiheng/phd/BeliefSpaceMotionPlanning/GaussianBeliefTrees/results/final/varyK/2d_simple_underwater/log/ompl_Vivaldi_2025-03-09 05:28:36.log")

analyze_progress_by_time_buckets(latest_file_rrbt)

# plot()

latest_file_rrbt = get_latest_file("results/final/rrbt/2d_simple_narrow/log/")

analyze_progress_by_time_buckets("/home/qiheng/phd/BeliefSpaceMotionPlanning/GaussianBeliefTrees/results/final/fixedK/2d_simple_narrow/log/ompl_Vivaldi_2025-03-09 03:46:17.log")

analyze_progress_by_time_buckets("/home/qiheng/phd/BeliefSpaceMotionPlanning/GaussianBeliefTrees/results/final/varyK/2d_simple_narrow/log/ompl_Vivaldi_2025-03-09 03:46:17.log")

analyze_progress_by_time_buckets(latest_file_rrbt)
# analyze_progress_by_time_buckets("results/final/rrbt/2d_simple_underwater/log/ompl_Vivaldi_2024-07-07 06:38:52.log")

# results/final/varyK/2d_simple_narrow/log/ompl_Vivaldi_2024-07-07 04:44:26.log
# main_costs, main_times, progress_costs, progress_iterations, progress_times = parse_data_file("results/final/rrbt/2d_simple_block/log/ompl_Vivaldi_2024-07-07 03:05:49.log")


time_buckets = [0.1, 0.2, 0.5, 1.0, 2.5, 5.0, 10.0, 20, 30, 40, 50, 60]
# Define the base path for results
base_path = "/home/qiheng/phd/BeliefSpaceMotionPlanning/GaussianBeliefTrees/results/final"

# Define environment names
environments = ["2d_simple_block", "2d_simple_narrow", "2d_simple_underwater"]

environments = ["2d_unicycle_underwater"]

environments = ["2d_double_block", "2d_double_narrow", "2d_double_underwater"]

# Define methods
methods = Dict(
    "fixedK" => "fixedK",
    "varyK" => "varyK",
    "RRBT" => "RRBT"
)

methods = Dict(
    "fixedK" => "fixedK",
    "varyK" => "varyK",
)

# Create individual plots for each environment
for env in environments
    # Define the methods and their log file paths for this environment
    env_methods = Dict()
    
    for (method_key, method_path) in methods
        if method_key == "RRBT"
            env_methods[method_key] = get_latest_file("$base_path/rrbt/$env/log/")
        elseif (method_key == "fixedK")
            env_methods[method_key] =   get_latest_file("$base_path/fixedK/$env/log/")
        else
            env_methods[method_key] = get_latest_file("$base_path/varyK/$env/log/")
        end
    end

    @show env_methods["varyK"]
    
    println("\nProcessing environment: $env")
    compare_methods_by_time_buckets(env, env_methods, time_buckets)
end

# Create individual plots for each environment
for env in environments
    # Define the methods and their log file paths for this environment
    env_methods = Dict()
    
    for (method_key, method_path) in methods
        if method_key == "RRBT"
            env_methods[method_key] = get_latest_file("$base_path/rrbt/$env/log/")
        elseif (method_key == "fixedK")
            env_methods[method_key] = get_latest_file("$base_path/fixedK/$env/log/")
        else
            env_methods[method_key] = get_latest_file("$base_path/varyK/$env/log/")
        end
    end
    
    println("\nProcessing environment: $env")
    analyze_first_solutions(env, env_methods)
end