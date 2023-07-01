use std::{io::{BufReader, BufRead}, fs::File, collections::{HashMap, HashSet}};

use nalgebra::{Rotation3, Matrix3, Vector3, ArrayStorage};

struct Scanner {
    beacons: Vec<Vector3<i16>>,
    diff_beacons: Vec<Vector3<i16>>,
    position: Vector3<i16>,
    rotation: Rotation3<i16>
}

const ROTATIONS: [Rotation3<i16>; 24] = [
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]]})), // O()
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 1]]})), // O(z)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 0]]})), // O(zx)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]]})), // O(zx2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 1, 0],
        [0, 0, -1],
        [-1, 0, 0]]})), // O(zx3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [-1, 0, 0],
        [0, -1, 0],
        [0, 0, 1]]})), // O(z2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [-1, 0, 0],
        [0, 0, 1],
        [0, 1, 0]]})), // O(z2x)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, -1]]})), // O(z2x2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [-1, 0, 0],
        [0, 0, -1],
        [0, -1, 0]]})), // O(z2x3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]]})), // O(z3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, -1, 0],
        [0, 0, 1],
        [-1, 0, 0]]})), // O(z3x)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1]]})), // O(z3x2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, -1, 0],
        [0, 0, -1],
        [1, 0, 0]]})), // O(z3x3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0]]})), // O(x)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]]})), // O(xz)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, 1],
        [0, -1, 0],
        [1, 0, 0]]})), // O(xzx)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0]]})), // O(xzx2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, 1],
        [0, 1, 0],
        [-1, 0, 0]]})), // O(xzx3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0]]})), // O(xz3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, -1],
        [0, -1, 0],
        [-1, 0, 0]]})), // O(xz3x)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0]]})), // O(xz3x2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [0, 0, -1],
        [0, 1, 0],
        [1, 0, 0]]})), // O(xz3x3)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]]})), // O(x2)
    Rotation3::from_matrix_unchecked(Matrix3::from_array_storage(ArrayStorage{0: [
        [1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]]}))  // O(x3)
    ];


fn recover_area(mut scanners: Vec<Scanner>) -> usize {

    let mut placed_scanners = vec![scanners.remove(0)];
    let mut all_beacons: Vec<Vector3<i16>> = Vec::new();

    placed_scanners[0].beacons.iter().for_each(|v| all_beacons.push(*v));

    let mut round = 0;

    while !scanners.is_empty() {

        let main_scanner = &placed_scanners[round % placed_scanners.len()];

        println!("main scanner first beacon: {:?}", main_scanner.beacons[0]);

        let mut candidate_found = false;

        for (candidate_index, candidate_scanner) in scanners.iter_mut().enumerate() {

            // println!("candidate scanner first beacon: {:?}", candidate_scanner.beacons[0]);

            for rotation in ROTATIONS {

                let mut position_candidates: HashMap<Vector3<i16>, HashSet<(usize, usize)>> = HashMap::new();

                for (candidate_diff_index, candidate_diff_vector) in candidate_scanner.diff_beacons.iter().enumerate().filter(|(i, _)| i / candidate_scanner.beacons.len() != i % candidate_scanner.beacons.len()) {

                    for (main_diff_index, _) in main_scanner.diff_beacons.iter().enumerate().filter(
                        |&(_, v)| main_scanner.rotation * v != rotation * candidate_diff_vector
                    ) {
                        let main_beacon_index1 = main_diff_index / main_scanner.beacons.len();
                        let candidate_beacon_index1 = candidate_diff_index / candidate_scanner.beacons.len();

                        let main_beacon_index2 = main_diff_index % main_scanner.beacons.len();
                        let candidate_beacon_index2 = candidate_diff_index % candidate_scanner.beacons.len();

                        let position_candidate1 = 
                            main_scanner.position + main_scanner.rotation * main_scanner.beacons[main_beacon_index1]
                            -rotation * candidate_scanner.beacons[candidate_beacon_index1];

                        let position_candidate2 = 
                            main_scanner.position + main_scanner.rotation * main_scanner.beacons[main_beacon_index2]
                            -rotation * candidate_scanner.beacons[candidate_beacon_index2];
                        
                        if let Some(value) = position_candidates.get_mut(&position_candidate1) {
                            value.insert((main_beacon_index1, candidate_beacon_index1));
                        } else {
                            let mut new_set = HashSet::new();
                            new_set.insert((main_beacon_index1, candidate_beacon_index1));
                            position_candidates.insert(position_candidate1, new_set);
                        }

                        if let Some(value) = position_candidates.get_mut(&position_candidate2) {
                            value.insert((main_beacon_index2, candidate_beacon_index2));
                        } else {
                            let mut new_set = HashSet::new();
                            new_set.insert((main_beacon_index2, candidate_beacon_index2));
                            position_candidates.insert(position_candidate2, new_set);
                        }
                    }
                }

                if let Some((best_position_candidate, max_matches)) = position_candidates.iter().max_by(
                    |(_, v1), (_, v2)|
                            v1.len().cmp(&v2.len())) {

                    // println!("rotation: {:?}, max matches {} for position {:?}", rotation, max_matches.len(), best_position_candidate);
                    if max_matches.len() >= 12 {
                        candidate_scanner.rotation = rotation;
                        candidate_scanner.position = *best_position_candidate;

                        candidate_scanner.beacons.iter().map(|v| candidate_scanner.position + rotation * v)
                        .for_each(|v| if !all_beacons.contains(&v) {all_beacons.push(v)});
    
                        candidate_found = true;
    
                        break;
                    }                
                }
            }

            if candidate_found {
                placed_scanners.push(scanners.remove(candidate_index));

                println!("Found overlap {} out of {}", placed_scanners.len(), scanners.len() + placed_scanners.len());

                break;
            }
        }

        round += 1;
    }

    //println!("all beacons:");
    //all_beacons.iter().for_each(|v| println!("beacon {:?}", v));
    //all_beacons.len()

    println!("all scanners:");
    placed_scanners.iter().for_each(|s| println!("Scanner at {:?}", s.position));

    let mut max_man_dist = 0;

    for (index_scanner_a, scanner_a) in placed_scanners.iter().enumerate() {
        for scanner_b in placed_scanners[index_scanner_a..].iter() {
            max_man_dist = std::cmp::max(max_man_dist, (scanner_a.position - scanner_b.position).abs().sum());
        }
    }

    max_man_dist.try_into().unwrap()
}


fn main() {

    let mut scanners: Vec<Scanner> = Vec::new();    

    let lines = BufReader::new(File::open("input.txt").unwrap()).lines().map(|l| l.unwrap());

    for line in lines {

        let splits: Vec<&str> = line.split(',').collect();

        if splits.len() == 1 && !line.is_empty() {
            scanners.push(Scanner { beacons: Vec::new(), diff_beacons: Vec::new(), position: Vector3::zeros(), rotation: ROTATIONS[0] });

        } else if splits.len() > 1 {
            scanners.last_mut().unwrap().beacons.push(Vector3::from_iterator(splits.iter().map(|v| v.parse::<i16>().unwrap())));
        }
    }

    for scanner in scanners.iter_mut() {
        let mut diff = Vec::with_capacity(scanner.beacons.len() * scanner.beacons.len());

        for beacon_a in scanner.beacons.iter() {
            for beacon_b in scanner.beacons.iter() {
                diff.push(beacon_a - beacon_b);
            }
        }

        scanner.diff_beacons = diff;
    }

    println!("beacons {}", recover_area(scanners));


}