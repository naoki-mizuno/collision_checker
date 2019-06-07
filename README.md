# collision_checker

Collision checker for an arbitrary robot shape.

## Speed

With 8 threads, robot footprint creation (i.e. preprocessing) is done in
1.198us. Average time for one check over 20 trials was 38.927us. For this, a
robot of size 5m by 2m was used in a map with resolution 0.2m/grid, and the
yaw resolution was set to 0.1deg.

## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
