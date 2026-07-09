package frc.lib.simulation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

/**
 * A version of {@link Arena2026Rebuilt} that caps the number of fuel game pieces placed on the
 * field. The stock arena (even in efficiency mode) still spawns 100+ fuel pieces, which tanks sim
 * performance on slower machines. This trims the field down to {@link #MAX_FIELD_GAME_PIECES}
 * pieces, chosen at random from the normal layout, right after the base class places them.
 */
public class LightFieldArena extends Arena2026Rebuilt {
  private static final int MAX_FIELD_GAME_PIECES = 120;

  public LightFieldArena(boolean addRampCollider) {
    super(addRampCollider);
    setEfficiencyMode(true);
  }

  @Override
  public void placeGamePiecesOnField() {
    super.placeGamePiecesOnField();

    List<GamePieceOnFieldSimulation> fieldPieces = new ArrayList<>(gamePiecesOnField());
    Collections.shuffle(fieldPieces);

    for (int i = MAX_FIELD_GAME_PIECES; i < fieldPieces.size(); i++) {
      removeGamePiece(fieldPieces.get(i));
    }
  }
}
