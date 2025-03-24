import pandas as pd

def load_algo_data(algo_names, map_name):
    dfs = []
    for name in algo_names:
        df = pd.read_csv(f"metrics/{name}_{map_name}_metrics.csv")
        df["Algorithm"] = name
        dfs.append(df)
    return pd.concat(dfs, ignore_index=True)

def find_valid_episodes(combined_df, algo_name, other_algo_names):
    fg_drm_success = combined_df[
        (combined_df["Algorithm"] == algo_name) &
        (combined_df["Outcome"] == "success")
    ]["Episode"].unique()
    
    # 创建透视表用于比较
    pivot_df = combined_df.pivot(
        index="Episode",
        columns="Algorithm",
        values=["Episodic Time Cost", "Episodic Dist Cost"]
    )
    
    valid_episodes = []
    
    for episode in fg_drm_success:
        try:
            fg_drm_time = pivot_df.loc[episode, ("Episodic Time Cost", "fg_drm")]
            fg_drm_dist = pivot_df.loc[episode, ("Episodic Dist Cost", "fg_drm")]
            
            # 比较其他算法
            time_valid = all(
                fg_drm_time < pivot_df.loc[episode, ("Episodic Time Cost", algo)]
                for algo in other_algo_names
            )
            
            dist_valid = all(
                fg_drm_dist < pivot_df.loc[episode, ("Episodic Dist Cost", algo)]
                for algo in other_algo_names
            )
            
            if time_valid or dist_valid:
                valid_episodes.append(episode)
                
        except KeyError:
            continue
            
    return valid_episodes

def generate_report(combined_df, valid_episodes):
    # 筛选有效episode的数据
    report_df = combined_df[combined_df["Episode"].isin(valid_episodes)]
    
    # 选择需要的列并重命名
    columns_mapping = {
        "Algorithm": "Algo Name",
        "Outcome": "Outcome",
        "Episodic Time Cost": "Episodic Time Cost",
        "Episodic Dist Cost": "Episodic Dist Cost",
        "Success Rate": "Success Rate"
    }
    
    # 生成最终格式
    formatted_df = report_df[["Episode", "Algorithm", "Outcome", 
                            "Episodic Time Cost", "Episodic Dist Cost",
                            "Success Rate"]].rename(columns=columns_mapping)
    
    # 按Episode和算法名称排序
    return formatted_df.sort_values(["Episode", "Algo Name"]).reset_index(drop=True)

if __name__ == "__main__":
    map_name = "maze_dense"
    algo = "fg_drm"
    other_algos = ["dprm_planner", "dynamic_prm", "dynamic_ug"]
    algorithms = [algo] + other_algos
    
    # 数据加载
    combined_data = load_algo_data(algorithms, map_name)
    
    # 查找有效episode
    valid_eps = find_valid_episodes(combined_data, algo_name = algo, other_algo_names = other_algos)
    
    # 生成最终报告
    result_df = generate_report(combined_data, valid_eps)
    
    print("\nCandidate episodes for illustration: ")
    print(result_df.to_string(index=False))