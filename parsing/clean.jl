using StatsPlots

function plot_bias_comparison(all_data, environment_name)
    sorted_data = sort(all_data, by = x -> x.bias)
    labels = [string(round(d.bias, digits=3)) for d in sorted_data]
    all_times = [d.times for d in sorted_data]
    all_costs = [d.costs for d in sorted_data]

    p_time = boxplot(
        labels,
        all_times;
        title="Time to First Solution by Bias - $environment_name",
        ylabel="Time (s)",
        xlabel="Bias Value",
        legend=false,
        fillalpha=0.6,
        linewidth=1.5,
        outliers=true,
        size=(900, 500),
        xrotation=45
    )

    p_cost = boxplot(
        labels,
        all_costs;
        title="First Solution Cost by Bias - $environment_name",
        ylabel="Cost",
        xlabel="Bias Value",
        legend=false,
        fillalpha=0.6,
        linewidth=1.5,
        outliers=true,
        size=(900, 500),
        xrotation=45
    )

    p_combined = plot(p_time, p_cost, layout=(1, 2), size=(1600, 600))
    savefig(p_combined, "bias_comparison_$environment_name.png")
    return p_combined
end
