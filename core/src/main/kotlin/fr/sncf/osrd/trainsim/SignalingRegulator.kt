package fr.sncf.osrd.trainsim

import fr.sncf.osrd.api.FullInfra
import fr.sncf.osrd.path.interfaces.TrainPath
import fr.sncf.osrd.standalone_sim.PathSignal

// TODO: BAL only for now
private val STOP_ASPECTS = setOf("S", "C")

class SignaledTrain(
    val simulator: TrainSimulator,
    val departure: PreciseDuration = 0.microseconds,
    val path: TrainPath,
    val length: PreciseDistance,
    val baseConstraints: List<Constraint>,
) {
    private val holds = mutableMapOf<PathSignal, HoldAt>()

    internal fun holdAt(signal: PathSignal): Constraint =
        holds.getOrPut(signal) { HoldAt(signal.pathOffset.toPrecise()) }
}

class SignalingRegulator(
    private val fullInfra: FullInfra,
    private val trains: List<SignaledTrain>,
) : Regulator {
    fun scheduledTrains(): List<ScheduledTrain> = trains.map {
        ScheduledTrain(it.simulator, it.departure)
    }

    override fun regulate() {
        val occupied =
            trains
                .flatMap {
                    occupiedZones(it.path.getZoneRanges(), it.simulator.state.position, it.length)
                }
                .toSet()

        for (train in trains) {
            train.simulator.constraints = train.baseConstraints
        }
        for (train in trains.filter { !it.simulator.isFinished }) {
            val states = signalStates(fullInfra, train.path, occupied)
            val ahead =
                signalsAhead(
                    fullInfra.blockInfra,
                    train.path,
                    train.simulator.state.position,
                    states,
                )
            val stop = ahead.entries.firstOrNull { it.value.getEnum("aspect") in STOP_ASPECTS }
            if (stop != null) {
                train.simulator.constraints = train.baseConstraints + train.holdAt(stop.key)
            }
        }
    }
}
