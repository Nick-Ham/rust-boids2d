use glam::Vec2;
use rand::{seq::IteratorRandom, thread_rng, Rng};
use raylib::prelude::*;

#[derive(Copy, Clone)]
struct Boid {
    position: Vec2,
    direction: Vec2,
}

const WINDOW_WIDTH: i32 = 640;
const WINDOW_HEIGHT: i32 = 480;
const WINDOW_COLOR: Color = Color::WHITE;

const BOIDS_COUNT: i32 = 250;
const BOID_SIZE: i32 = 2;
const BOID_COLOR: Color = Color::BLACK;
const BOID_SPEED: f32 = 170.0;

const BOID_NEIGHBOR_RADIUS: f32 = 25.0;
const BOID_PROTECTED_RADIUS: f32 = 5.0;
const BOID_SEPARATION_SCALAR: f32 = 10.0; //4.0;
const BOID_ALIGNMENT_SCALAR: f32 = 1.5; //2.0;
const BOID_COHESION_SCALAR: f32 = 3.9; //3.0;
const BOID_GOAL_SCALAR: f32 = 1.9;
const BOID_GOAL: Vec2 = Vec2::new(320.0, 240.0);

const _AFFECTOR_DISABLE_TOLERANCE: f32 = 0.01;

const SHOULD_DRAW_OPTIMIZED_BOIDS: bool = false;
const SHOULD_DRAW_NEIGHBOR_RADIUS: bool = false;
const SHOULD_DRAW_PROTECTED_RADIUS: bool = false;
const SHOULD_DRAW_DIRECTION: bool = false;

const DEBUG_DIRECTION_LENGTH: f32 = 5.0;

fn main() {
    launch_boids()
}

fn launch_boids() {
    let mut rng = thread_rng();
    let mut boids: Vec<Boid> = Vec::new();

    for _ in 0..BOIDS_COUNT {
        let rand_x_position = rng.gen_range(1..WINDOW_WIDTH) as f32;
        let rand_y_position = rng.gen_range(1..WINDOW_HEIGHT) as f32;

        let _center_x_position = WINDOW_WIDTH as f32 / 2.0;
        let _center_y_position = WINDOW_HEIGHT as f32 / 2.0;

        let rand_rad = rng.gen_range(0.0..{ PI * 2.0 }) as f32;
        let rand_direction = Vec2::new(rand_rad.cos(), rand_rad.sin());

        let new_boid = Boid {
            position: Vec2::new(rand_x_position, rand_y_position),
            direction: rand_direction,
        };

        boids.push(new_boid);
    }

    let (mut rl, thread) = raylib::init()
        .size(WINDOW_WIDTH, WINDOW_HEIGHT)
        .title("Hello, World")
        .build();

    let mut frame_start = std::time::Instant::now();
    let mut frame_duration = 0.0;
    while !rl.window_should_close() {
        for boids_index in 0..boids.len() {
            let mut neighboring_boids: Vec<Boid> = Vec::new();
            let mut neighboring_boids_protected: Vec<Boid> = Vec::new();
            let mut neighboring_boids_protected_distance_squared: Vec<f32> = Vec::new();
            for boids_compare_index in 0..boids.len() {
                if boids_index == boids_compare_index {
                    continue;
                }

                let distance_to_neighbor_squared = (boids[boids_compare_index].position
                    - boids[boids_index].position)
                    .length_squared();

                if distance_to_neighbor_squared > BOID_NEIGHBOR_RADIUS.powf(2.0) {
                    continue;
                }

                if distance_to_neighbor_squared < BOID_PROTECTED_RADIUS.powf(2.0) {
                    neighboring_boids_protected.push(boids[boids_compare_index]);
                    neighboring_boids_protected_distance_squared.push(distance_to_neighbor_squared);
                }

                neighboring_boids.push(boids[boids_compare_index]);
            }

            let mut cohesion_direction = Vec2::new(0.0, 0.0);
            let mut separation_direction = Vec2::new(0.0, 0.0);
            let mut alignment_direction = Vec2::new(0.0, 0.0);
            let mut goal_direction = Vec2::new(0.0, 0.0);

            // Cohesion
            if !neighboring_boids.is_empty() {
                let mut average_position_neighboring: Vec2 = boids[boids_index].position;
                for neighboring_boid in neighboring_boids.iter() {
                    average_position_neighboring += neighboring_boid.position;
                }
                average_position_neighboring /= neighboring_boids.len() as f32 + 1.0;
                cohesion_direction = (average_position_neighboring - boids[boids_index].position)
                    .normalize()
                    * BOID_COHESION_SCALAR
                    * frame_duration;
            }

            // Separation
            if !neighboring_boids_protected.is_empty() {
                let mut nearest_boid = neighboring_boids_protected[0];
                let mut minimum_distance = neighboring_boids_protected_distance_squared[0];
                for protected_boid_index in 1..neighboring_boids_protected.len() {
                    if neighboring_boids_protected_distance_squared[protected_boid_index]
                        < minimum_distance
                    {
                        nearest_boid = neighboring_boids_protected[protected_boid_index];
                        minimum_distance =
                            neighboring_boids_protected_distance_squared[protected_boid_index];
                    }
                }
                separation_direction = (boids[boids_index].position - nearest_boid.position)
                    .normalize()
                    * BOID_SEPARATION_SCALAR
                    * frame_duration;
            }

            // Alignment
            if !neighboring_boids.is_empty() {
                let mut average_direction: Vec2 = boids[boids_index].direction;
                for neighboring_boid in neighboring_boids.iter() {
                    average_direction += neighboring_boid.direction;
                }
                average_direction /= neighboring_boids.len() as f32 + 1.0;
                alignment_direction =
                    average_direction.normalize() * BOID_ALIGNMENT_SCALAR * frame_duration;
            }

            // Goal
            goal_direction = (BOID_GOAL - boids[boids_index].position).normalize()
                * BOID_GOAL_SCALAR
                * frame_duration;

            // Affector
            boids[boids_index].direction = (boids[boids_index].direction
                + cohesion_direction
                + separation_direction
                + alignment_direction
                + goal_direction)
                .normalize();

            let velocity = boids[boids_index].direction * BOID_SPEED * frame_duration;
            boids[boids_index].position += velocity;
        }

        let mut d = rl.begin_drawing(&thread);

        d.clear_background(WINDOW_COLOR);

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

        frame_duration = frame_start.elapsed().as_secs_f32();
        frame_start = std::time::Instant::now();
    }
}
