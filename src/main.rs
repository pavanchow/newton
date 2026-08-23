use clap::{Parser, Subcommand};
use newton::{Body, Bounds, Vec2, World};

#[derive(Parser)]
#[command(name = "newton", about = "A 2D rigid-body physics engine, from scratch")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run a small demo scene for N steps and print a summary.
    Run {
        #[arg(long, default_value_t = 300)]
        steps: u32,
        #[arg(long, default_value_t = 1.0 / 60.0)]
        dt: f64,
    },
}

fn build_demo_scene() -> World {
    let bounds = Bounds {
        min: Vec2::new(0.0, 0.0),
        max: Vec2::new(20.0, 20.0),
    };
    let mut world = World::new(Vec2::new(0.0, -9.8), bounds);

    world.add_body(Body::new(Vec2::new(6.0, 18.0), 1.0, 1.0, 0.6));
    world.add_body(Body::new(Vec2::new(6.0, 14.0), 1.0, 1.0, 0.6));
    world.add_body(Body::new(Vec2::new(14.0, 16.0), 1.5, 2.0, 0.4));
    world.add_body(Body::new(Vec2::new(10.0, 4.0), 1.0, 1.0, 0.3));

    world
}

fn main() {
    let cli = Cli::parse();

    match cli.command {
        Command::Run { steps, dt } => {
            let mut world = build_demo_scene();

            for _ in 0..steps {
                world.step(dt);
            }

            println!("newton: ran {} steps at dt={:.5}", steps, dt);
            for (i, body) in world.bodies.iter().enumerate() {
                println!(
                    "  body {i}: pos=({:.3}, {:.3}) vel=({:.3}, {:.3}) radius={:.2} mass={:.2} restitution={:.2}",
                    body.position.x,
                    body.position.y,
                    body.velocity.x,
                    body.velocity.y,
                    body.radius,
                    body.mass,
                    body.restitution
                );
            }
        }
    }
}
