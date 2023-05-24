use rayon::prelude::*;

use glam::{IVec2, Vec2};
use rand::{thread_rng, Rng};
use raylib::prelude::*;

pub struct Boid {
    position: Vec2,
    direction: Vec2,
}

const WINDOW_DIMS: IVec2 = IVec2 { x: 1280, y: 720 };
const WINDOW_CENTER: IVec2 = IVec2 {
    x: WINDOW_DIMS.x / 2,
    y: WINDOW_DIMS.y / 2,
};
const WINDOW_COLOR: Color = Color::WHITE;

const BOIDS_COUNT: i32 = 1024;
const BOID_SIZE: i32 = 3;
const BOID_COLOR: Color = Color::BLACK;

const BOID_NEIGHBOR_RADIUS: f32 = 10.0;
const BOID_PROTECTED_RADIUS: f32 = 5.0;
const MAX_NEIGHBOR_POPULATION: usize = 20;

const BOID_SEPARATION_SCALAR: f32 = 0.1;
const BOID_ALIGNMENT_SCALAR: f32 = 0.02;
const BOID_COHESION_SCALAR: f32 = 0.05;
const BOID_SPEED: f32 = 2.0;

const BOID_GOAL_SCALAR: f32 = 0.02;
const BOID_GOAL: Vec2 = Vec2 {
    x: WINDOW_CENTER.x as f32,
    y: WINDOW_CENTER.y as f32,
};

const _AFFECTOR_DISABLE_TOLERANCE: f32 = 0.01;

const SHOULD_DRAW_OPTIMIZED_BOIDS: bool = false;
const SHOULD_DRAW_NEIGHBOR_RADIUS: bool = false;
const SHOULD_DRAW_PROTECTED_RADIUS: bool = false;
const SHOULD_DRAW_DIRECTION: bool = false;

const DEBUG_DIRECTION_LENGTH: f32 = 5.0;

const FRAMES_PER_SECOND: u32 = 120;
const TIMESTEP: f32 = 1.0 / FRAMES_PER_SECOND as f32;

fn main() {
    let mut boids: Vec<Boid> = Vec::new();
    make_boids(&mut boids);

    let (mut rl, thread) = raylib::init()
        .size(WINDOW_DIMS.x, WINDOW_DIMS.y)
        .title("Hello, World")
        .build();

    let mut time_since_last_update = 0.0;
    while !rl.window_should_close() {
        let dt = rl.get_frame_time();
        time_since_last_update += dt;
        while time_since_last_update > TIMESTEP {
            time_since_last_update -= TIMESTEP;
            step_boids_parallel(&mut boids);
        }
        draw_boids(&mut rl, &thread, &boids);
    }
}

pub fn make_boids(boids: &mut Vec<Boid>) {
    let mut rng = thread_rng();
    for _ in 0..BOIDS_COUNT {
        let new_boid = Boid {
            position: Vec2::new(
                rng.gen_range(1.0..WINDOW_DIMS.x as f32),
                rng.gen_range(1.0..WINDOW_DIMS.y as f32),
            ),
            direction: Vec2 {
                x: rng.gen_range(-1.0..=1.0),
                y: rng.gen_range(-1.0..=1.0),
            }
            .normalize(),
        };
        boids.push(new_boid);
    }
}

fn step_boids_parallel(boids: &mut Vec<Boid>) {
    let deltas: Vec<Vec2> = boids
        .par_iter()
        .map(|boid| {
            let mut neighboring_boids: Vec<&Boid> = Vec::new();
            let mut current_min: f32 = f32::MAX;
            let mut closest_boid = None;

            for other_boid in boids.iter() {
                if std::ptr::eq(boid, other_boid) {
                    continue;
                }
                let dist = (other_boid.position - boid.position).length();
                if dist > BOID_NEIGHBOR_RADIUS {
                    continue;
                }
                if dist < current_min {
                    closest_boid = Some(other_boid);
                }
                neighboring_boids.push(other_boid);
            }
            calc_boids_delta(&boid, &neighboring_boids, &closest_boid)
        })
        .collect::<Vec<_>>();

    for (boid, delta) in boids.iter_mut().zip(deltas.into_iter()) {
        boid.direction = (boid.direction + delta).normalize();
        let velocity = boid.direction * BOID_SPEED;
        boid.position += velocity;
    }
}

fn step_boids(boids: &mut Vec<Boid>) {
    for i in 0..boids.len() {
        let mut neighboring_boids: Vec<&Boid> = Vec::new();

        let boid = &boids[i];
        let mut current_min: f32 = f32::MAX;
        let mut closest_boid = None;
        for other_boid in boids.iter() {
            if std::ptr::eq(boid, other_boid) {
                continue;
            }
            let dist = (other_boid.position - boid.position).length();
            if dist > BOID_NEIGHBOR_RADIUS {
                continue;
            }
            if dist < current_min {
                closest_boid = Some(other_boid);
            }
            neighboring_boids.push(other_boid);
        }

        let delta = calc_boids_delta(&boid, &neighboring_boids, &closest_boid);

        let boid = &mut boids[i];
        boid.direction = (boid.direction + delta).normalize();
        let velocity = boid.direction * BOID_SPEED;
        boid.position += velocity;
    }
}

pub fn calc_boids_delta(
    boid: &Boid,
    neighboring_boids: &Vec<&Boid>,
    closest_boid: &Option<&Boid>,
) -> Vec2 {
    let mut cohesion_direction = Vec2::new(0.0, 0.0);
    let mut separation_direction = Vec2::new(0.0, 0.0);
    let mut alignment_direction = Vec2::new(0.0, 0.0);

    // Cohesion
    if !neighboring_boids.is_empty() {
        let mut average_position_neighboring: Vec2 = boid.position;
        for neighboring_boid in neighboring_boids.iter() {
            average_position_neighboring += neighboring_boid.position;
        }
        average_position_neighboring /= neighboring_boids.len() as f32 + 1.0;
        cohesion_direction =
            (average_position_neighboring - boid.position).normalize() * BOID_COHESION_SCALAR;
    }

    if let Some(closest_boid) = closest_boid {
        let delta = (boid.position - closest_boid.position);
        let dist_to_closest = delta.length();
        if dist_to_closest < BOID_PROTECTED_RADIUS {
            separation_direction = delta.normalize() * BOID_SEPARATION_SCALAR;
        }
    }

    // Alignment
    if !neighboring_boids.is_empty() {
        let mut average_direction: Vec2 = boid.direction;
        for neighboring_boid in neighboring_boids.iter() {
            average_direction += neighboring_boid.direction;
        }
        average_direction /= neighboring_boids.len() as f32 + 1.0;
        alignment_direction = average_direction.normalize() * BOID_ALIGNMENT_SCALAR;
    }

    // Goal
    let goal_direction = (BOID_GOAL - boid.position).normalize() * BOID_GOAL_SCALAR;
    cohesion_direction + separation_direction + alignment_direction + goal_direction
}

pub fn draw_boids(rl: &mut RaylibHandle, thread: &RaylibThread, boids: &Vec<Boid>) {
    let mut d = rl.begin_drawing(&thread);

    // d.clear_background(WINDOW_COLOR);
    d.draw_rectangle(
        0,
        0,
        WINDOW_DIMS.x,
        WINDOW_DIMS.y,
        Color::new(255, 255, 255, 1),
    );
    for boid in boids.as_slice() {
        if SHOULD_DRAW_NEIGHBOR_RADIUS {
            d.draw_circle_lines(
                boid.position.x as i32,
                boid.position.y as i32,
                BOID_NEIGHBOR_RADIUS,
                Color::GREEN,
            );
        }
        if SHOULD_DRAW_PROTECTED_RADIUS {
            d.draw_circle_lines(
                boid.position.x as i32,
                boid.position.y as i32,
                BOID_PROTECTED_RADIUS,
                Color::RED,
            );
        }
        if SHOULD_DRAW_DIRECTION {
            d.draw_line(
                boid.position.x as i32,
                boid.position.y as i32,
                (boid.position.x + boid.direction.x * DEBUG_DIRECTION_LENGTH) as i32,
                (boid.position.y + boid.direction.y * DEBUG_DIRECTION_LENGTH) as i32,
                Color::BLUE,
            )
        }
        if SHOULD_DRAW_OPTIMIZED_BOIDS {
            d.draw_pixel(boid.position.x as i32, boid.position.y as i32, BOID_COLOR);
        } else {
            d.draw_rectangle(
                (boid.position.x - BOID_SIZE as f32 / 2.0) as i32,
                (boid.position.y - BOID_SIZE as f32 / 2.0) as i32,
                BOID_SIZE,
                BOID_SIZE,
                BOID_COLOR,
            )
        }
    }
}
