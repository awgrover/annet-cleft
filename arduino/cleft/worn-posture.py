#!/usr/bin/env perl
# Generate pre-calced values for positions

use strict; use warnings; no warnings 'uninitialized'; use 5.012; no if ($^V ge v5.18.0), warnings => 'experimental::smartmatch';
use Carp; $SIG{__DIE__} = sub { Carp::confess @_ };

use Math::Trig;

sub do_worn_posture {
    # as if draped over shoulders,
    # with "opening" at rear
    # So, the rear is low, the side high, the front low
    # "half" the segments: notionally 1..7 (8 dups 7) mirror for 9..15
    # cos() for smoothness
    # magnitude -1..1

    my $midpoint_bias = 0.8; # as in, bias downwards
    my $max_height = 10; # cm, probably less?
    my @magnitudes;

    for my $segment ((0..6)) {
        my $theta = -pi()/2 + $segment/6 * pi();
        my $magnitude = cos($theta) - $midpoint_bias;
        printf("[%d] %6.3f %5.1f -> %4.2f %4.1fcm\n", $segment, $theta, $theta * 180 / pi(), $magnitude, $magnitude * $max_height);
        push @magnitudes, $magnitude;
        }

    say "";
    say "// generated from: ",$0;
    printf("// cos(-pi/2..pi/2) ct=%d mid=%3.1f %s\n", scalar(@magnitudes), $midpoint_bias, "".localtime() );
    printf "const float AnimationWornPosture::MidPointBias = %3.1f\n", $midpoint_bias;

    print("const float AnimationWornPosture::positions[7] = {");
    print( join ",", map {sprintf("%6.3f", $_)} @magnitudes);
    say("};");
    }

sub main {
    if ($ARGV[0] eq '' || $ARGV[0] eq 'worn-posture') {
        do_worn_posture();
    }
}

main();
