use std::collections::HashSet;
use std::fs::{File, read_to_string};
use std::io::Write;
use std::path::Path;
use std::time::Instant;

#[derive(Debug, Clone, PartialEq)]
struct Tree {
    id: u32,
    x: u32,
    y: u32,
}

#[derive(Debug, Clone)]
struct ChargingStation {
    _id: u32,
    x: u32,
    y: u32,
}

#[derive(Debug, Clone)]
struct Route {
    trees: Vec<Tree>,
    charging_station: ChargingStation,
}

macro_rules! scan {
    ($it:expr, $($t:ty),*) => {
        ($( $it.next().expect("Unexpected EOF").parse::<$t>().expect("Parse error") ),*)
    };
}

fn distance(t1: &Tree, t2: &Tree) -> f64 {
    let (x1, y1) = (t1.x as f64, t1.y as f64);
    let (x2, y2) = (t2.x as f64, t2.y as f64);
    ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
}

fn length(route: &[Tree]) -> f64 {
    if route.is_empty() {
        return 0.0;
    }
    let mut total = 0.0;
    for i in 0..route.len() - 1 {
        total += distance(&route[i], &route[i + 1]);
    }
    total
}

fn length_with_station(route: &[Tree], station: &ChargingStation) -> f64 {
    if route.is_empty() {
        return 0.0;
    }
    let station_tree = Tree {
        id: 0,
        x: station.x,
        y: station.y,
    };
    distance(&station_tree, &route[0])
        + length(route)
        + distance(&route[route.len() - 1], &station_tree)
}

fn length_with_return(route: &[Tree]) -> f64 {
    if route.is_empty() {
        return 0.0;
    }
    length(route) + distance(&route[route.len() - 1], &route[0])
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let in_path = args
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("resources/roboter11.txt");
    let out_path = if args.len() > 2 {
        args[2].clone()
    } else {
        let path = Path::new(in_path);
        let parent = path.parent().unwrap_or_else(|| Path::new("."));
        let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("out");
        parent
            .join(format!("{}_out.txt", stem))
            .to_string_lossy()
            .to_string()
    };

    let data = read_to_string(in_path).expect("Failed to read file");
    let mut it = data.split_whitespace();

    let max_time = scan!(it, u32) as f64;
    let tree_count = scan!(it, u32);
    let mut trees = Vec::new();

    for _ in 0..tree_count {
        let (id, x, y) = scan!(it, u32, u32, u32);
        trees.push(Tree { id, x, y });
    }

    let start = Instant::now();
    let mut routes = solve_mst(trees.clone(), max_time);
    let elapsed_solve = start.elapsed();

    let start_merge = Instant::now();
    merge_routes(&mut routes, max_time);
    let elapsed_merge = start_merge.elapsed();

    let start_2opt = Instant::now();
    two_opt(&mut routes);
    let elapsed_2opt = start_2opt.elapsed();

    let start_or_opt = Instant::now();
    or_opt(&mut routes, max_time);
    let elapsed_or_opt = start_or_opt.elapsed();

    let mut total_final = 0.0;
    for route in &routes {
        total_final += length_with_station(&route.trees, &route.charging_station);
    }

    println!("Routes: {}", routes.len());
    println!("Total length: {:.2}", total_final);
    println!("\nTiming:");
    println!("Algorithm (MST):     {:.4}s", elapsed_solve.as_secs_f64());
    println!("Route Merging:       {:.4}s", elapsed_merge.as_secs_f64());
    println!("2-Opt Optimization:  {:.4}s", elapsed_2opt.as_secs_f64());
    println!("Or-Opt Optimization: {:.4}s", elapsed_or_opt.as_secs_f64());
    println!(
        "Total execution:     {:.4}s",
        elapsed_solve.as_secs_f64()
            + elapsed_merge.as_secs_f64()
            + elapsed_2opt.as_secs_f64()
            + elapsed_or_opt.as_secs_f64()
    );
    write_output(&out_path, &routes, max_time).expect("Failed to write output");
    println!("\nOutput written to: {}", out_path);
}

fn write_output(path: &str, routes: &[Route], max_time: f64) -> std::io::Result<()> {
    let mut file = File::create(path)?;

    writeln!(file, "{}", routes.len())?;
    writeln!(file, "{}", max_time as u32)?;

    for route in routes {
        let route_length = length_with_station(&route.trees, &route.charging_station);
        writeln!(file, "{}", route_length.ceil() as u32)?;
        writeln!(
            file,
            "{} {}",
            route.charging_station.x, route.charging_station.y
        )?;

        let tree_ids: Vec<String> = route.trees.iter().map(|t| t.id.to_string()).collect();
        writeln!(file, "{}", tree_ids.join(" "))?;
    }

    Ok(())
}

fn solve_mst(trees: Vec<Tree>, max_time: f64) -> Vec<Route> {
    let mst = build_mst(&trees);
    let dfs_order = dfs(&mst, 0);

    let mut routes = Vec::new();
    let mut unvisited: HashSet<usize> = (0..trees.len()).collect();
    let mut occupied = HashSet::new();

    while !unvisited.is_empty() {
        let mut route = Vec::new();

        let start = *dfs_order
            .iter()
            .find(|&&idx| unvisited.contains(&idx))
            .unwrap();
        route.push(trees[start].clone());
        unvisited.remove(&start);

        for &idx in &dfs_order {
            if !unvisited.contains(&idx) {
                continue;
            }

            let mut candidate_route = route.clone();
            candidate_route.push(trees[idx].clone());
            let total_length = length_with_return(&candidate_route);
            if total_length <= max_time {
                route.push(trees[idx].clone());
                unvisited.remove(&idx);
            } else {
                break;
            }
        }

        let first_tree = &route[0];
        let cs_x = first_tree.x + 1;
        let mut cs_y = first_tree.y;

        while occupied.contains(&(cs_x, cs_y)) {
            cs_y += 1;
        }

        occupied.insert((cs_x, cs_y));

        let charging_station = ChargingStation {
            _id: routes.len() as u32,
            x: cs_x,
            y: cs_y,
        };

        routes.push(Route {
            trees: route,
            charging_station,
        });
    }

    routes
}

fn build_mst(trees: &[Tree]) -> Vec<Vec<usize>> {
    let n = trees.len();
    let mut mst = vec![Vec::new(); n];
    let mut in_mst = vec![false; n];
    let mut min_edge = vec![(f64::MAX, 0usize); n];

    min_edge[0] = (0.0, 0);

    for _ in 0..n {
        let u = (0..n)
            .filter(|&i| !in_mst[i])
            .min_by(|&a, &b| min_edge[a].0.partial_cmp(&min_edge[b].0).unwrap())
            .unwrap();

        in_mst[u] = true;

        if min_edge[u].1 != u {
            mst[min_edge[u].1].push(u);
            mst[u].push(min_edge[u].1);
        }

        for v in 0..n {
            if !in_mst[v] {
                let dist = distance(&trees[u], &trees[v]);
                if dist < min_edge[v].0 {
                    min_edge[v] = (dist, u);
                }
            }
        }
    }

    mst
}

fn dfs(mst: &[Vec<usize>], start: usize) -> Vec<usize> {
    let mut order = Vec::new();
    let mut visited = vec![false; mst.len()];
    _dfs(mst, start, &mut visited, &mut order);
    order
}

fn _dfs(mst: &[Vec<usize>], node: usize, visited: &mut [bool], order: &mut Vec<usize>) {
    visited[node] = true;
    order.push(node);

    for &neighbor in &mst[node] {
        if !visited[neighbor] {
            _dfs(mst, neighbor, visited, order);
        }
    }
}

fn merge_routes(routes: &mut Vec<Route>, max_time: f64) {
    loop {
        let mut merge_candidates: Vec<(usize, usize, f64)> = Vec::new();
        for i in 0..routes.len() {
            for j in (i + 1)..routes.len() {
                let mut combined = routes[i].trees.clone();
                combined.extend_from_slice(&routes[j].trees);
                let combined_length = length_with_return(&combined);

                if combined_length <= max_time {
                    let current_cost =
                        length_with_return(&routes[i].trees) + length_with_return(&routes[j].trees);
                    let saving = current_cost - combined_length;
                    merge_candidates.push((i, j, saving));
                }
            }
        }

        if merge_candidates.is_empty() {
            break;
        }

        merge_candidates.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

        let mut merged_indices = HashSet::new();
        let mut merges_this_round = 0;

        for (i, j, _) in merge_candidates {
            if merged_indices.contains(&i) || merged_indices.contains(&j) {
                continue;
            }

            let mut combined = routes[i].trees.clone();
            combined.extend_from_slice(&routes[j].trees);
            routes[i].trees = combined;

            routes[j].trees.clear();

            merged_indices.insert(i);
            merged_indices.insert(j);
            merges_this_round += 1;
        }

        routes.retain(|r| !r.trees.is_empty());
        if merges_this_round == 0 {
            break;
        }
    }
}

fn two_opt(routes: &mut [Route]) {
    let mut iteration = 0;

    'outer: loop {
        iteration += 1;

        for route in routes.iter_mut() {
            if route.trees.len() < 4 {
                continue;
            }
            for i in 0..route.trees.len() - 2 {
                for j in i + 2..route.trees.len() {
                    let current_dist = distance(&route.trees[i], &route.trees[i + 1])
                        + distance(&route.trees[j], &route.trees[(j + 1) % route.trees.len()]);
                    let new_dist = distance(&route.trees[i], &route.trees[j])
                        + distance(
                            &route.trees[i + 1],
                            &route.trees[(j + 1) % route.trees.len()],
                        );
                    if new_dist < current_dist {
                        route.trees[i + 1..=j].reverse();
                        if iteration > 100 {
                            break 'outer;
                        }
                        continue 'outer;
                    }
                }
            }
        }
        break;
    }
}

fn or_opt(routes: &mut Vec<Route>, max_time: f64) {
    let mut iteration = 0;

    'outer: loop {
        iteration += 1;
        for segment_size in 1..=3 {
            for i in 0..routes.len() {
                if routes[i].trees.len() <= segment_size {
                    continue;
                }

                for start_pos in 0..routes[i].trees.len() - segment_size + 1 {
                    let end_pos = start_pos + segment_size;
                    let segment: Vec<Tree> = routes[i].trees[start_pos..end_pos].to_vec();
                    let current_cost = length_with_return(&routes[i].trees);

                    let mut route_without = routes[i].trees.clone();
                    route_without.drain(start_pos..end_pos);
                    let cost_without = length_with_return(&route_without);

                    for j in 0..routes.len() {
                        if i == j {
                            continue;
                        }

                        for insert_pos in 0..=routes[j].trees.len() {
                            let mut route_with = routes[j].trees.clone();
                            route_with.splice(insert_pos..insert_pos, segment.iter().cloned());

                            let new_cost_i = cost_without;
                            let new_cost_j = length_with_return(&route_with);

                            if new_cost_i <= max_time * 1.000001 && new_cost_j <= max_time * 1.000001 {
                                let old_total = current_cost + length_with_return(&routes[j].trees);
                                let new_total = new_cost_i + new_cost_j;

                                if new_total < old_total - 1e-9 {
                                    routes[i].trees = route_without.clone();
                                    routes[j].trees = route_with;
                                    if iteration > 50 {
                                        break 'outer;
                                    }
                                    continue 'outer;
                                }
                            }
                        }
                    }
                }
            }
        }

        break;
    }

    routes.retain(|route| !route.trees.is_empty());
}
