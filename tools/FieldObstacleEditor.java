import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.JToolBar;
import javax.swing.SwingUtilities;
import java.awt.image.BufferedImage;

// Standalone Swing tool (plain Java, no WPILib) for drawing field obstacles over a field PNG and
// exporting them to obstacles.txt, which ObstacleAvoidance reads. Run it with the .bat.
public class FieldObstacleEditor extends JFrame {

  // Click within this many pixels of a shape's first point to close it.
  private static final int CLOSE_SNAP_PX = 12;

  private BufferedImage image;                              // the loaded field picture.
  private final List<List<Point>> shapes = new ArrayList<>(); // finished shapes (in image pixels).
  private List<Point> current = new ArrayList<>();          // the shape being drawn right now.

  // The field's real size in meters, typed by the user. The whole image maps to this.
  private final JTextField lengthField = new JTextField("16.54", 5);
  private final JTextField widthField = new JTextField("8.02", 5);
  private final JLabel status = new JLabel("Open a field image to start.");
  private final Canvas canvas = new Canvas();

  public FieldObstacleEditor() {
    super("Field Obstacle Editor");
    setDefaultCloseOperation(EXIT_ON_CLOSE);
    setLayout(new BorderLayout());

    JToolBar bar = new JToolBar();
    bar.setFloatable(false);
    addButton(bar, "Open Image", e -> openImage());
    bar.addSeparator();
    bar.add(new JLabel(" Field length(m): "));
    bar.add(lengthField);
    bar.add(new JLabel(" width(m): "));
    bar.add(widthField);
    bar.addSeparator();
    addButton(bar, "Close Shape", e -> closeShape());
    addButton(bar, "Undo Point", e -> undoPoint());
    addButton(bar, "Undo Shape", e -> undoShape());
    bar.addSeparator();
    addButton(bar, "Export", e -> export());

    add(bar, BorderLayout.NORTH);
    add(new JScrollPane(canvas), BorderLayout.CENTER);
    add(status, BorderLayout.SOUTH);

    setSize(1100, 750);
    setLocationRelativeTo(null);
  }

  private void addButton(JToolBar bar, String text, java.awt.event.ActionListener action) {
    JButton b = new JButton(text);
    b.addActionListener(action);
    bar.add(b);
  }

  private void openImage() {
    JFileChooser chooser = new JFileChooser();
    if (chooser.showOpenDialog(this) != JFileChooser.APPROVE_OPTION) {
      return;
    }
    try {
      image = ImageIO.read(chooser.getSelectedFile());
      shapes.clear();
      current = new ArrayList<>();
      canvas.setPreferredSize(new Dimension(image.getWidth(), image.getHeight()));
      canvas.revalidate();
      canvas.repaint();
      status.setText("Left-click to place points. Click the first point (or right-click) to close a shape.");
    } catch (IOException ex) {
      JOptionPane.showMessageDialog(this, "Could not read image: " + ex.getMessage());
    }
  }

  // Left-click handler: click near the first point to close the shape, otherwise add a point.
  private void onClick(Point p) {
    if (image == null) {
      return;
    }
    if (current.size() >= 3 && near(p, current.get(0))) {
      closeShape();
      return;
    }
    current.add(p);
    showCount();
    canvas.repaint();
  }

  private boolean near(Point a, Point b) {
    return Math.hypot(a.x - b.x, a.y - b.y) <= CLOSE_SNAP_PX;
  }

  private void closeShape() {
    if (current.size() >= 3) {
      shapes.add(current);
      current = new ArrayList<>();
      showCount();
      canvas.repaint();
    }
  }

  private void undoPoint() {
    if (!current.isEmpty()) {
      current.remove(current.size() - 1);
      canvas.repaint();
    }
  }

  private void undoShape() {
    if (!current.isEmpty()) {
      current.clear();
    } else if (!shapes.isEmpty()) {
      shapes.remove(shapes.size() - 1);
    }
    canvas.repaint();
  }

  private void showCount() {
    status.setText(shapes.size() + " shapes done, " + current.size() + " points in current shape.");
  }

  // Convert an image pixel to field meters. The whole image = the field, and the image's Y is
  // flipped (top of image = far side of field) to match WPILib's bottom-left origin.
  private double[] toField(Point p) {
    double length = parse(lengthField.getText(), 16.54);
    double width = parse(widthField.getText(), 8.02);
    double fx = (double) p.x / image.getWidth() * length;
    double fy = (double) (image.getHeight() - p.y) / image.getHeight() * width;
    return new double[] {fx, fy};
  }

  private static double parse(String s, double fallback) {
    try {
      return Double.parseDouble(s.trim());
    } catch (NumberFormatException e) {
      return fallback;
    }
  }

  private void export() {
    if (shapes.isEmpty()) {
      JOptionPane.showMessageDialog(this, "No sealed shapes to export.");
      return;
    }
    JFileChooser chooser = new JFileChooser();
    chooser.setSelectedFile(new File("../src/main/deploy/obstacles.txt"));
    if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) {
      return;
    }
    // Write each shape as: POLYGON, then "x y" meter vertices, then END (the format load() parses).
    try (FileWriter w = new FileWriter(chooser.getSelectedFile())) {
      w.write("# Generated by FieldObstacleEditor. Units: meters, field coordinates.\n");
      for (List<Point> shape : shapes) {
        w.write("POLYGON\n");
        for (Point p : shape) {
          double[] f = toField(p);
          w.write(String.format("%.4f %.4f%n", f[0], f[1]));
        }
        w.write("END\n");
      }
      status.setText("Exported " + shapes.size() + " shapes to " + chooser.getSelectedFile().getName());
    } catch (IOException ex) {
      JOptionPane.showMessageDialog(this, "Export failed: " + ex.getMessage());
    }
  }

  private class Canvas extends JPanel {
    Canvas() {
      setBackground(Color.DARK_GRAY);
      MouseAdapter m = new MouseAdapter() {
        @Override
        public void mousePressed(MouseEvent e) {
          if (SwingUtilities.isLeftMouseButton(e)) {
            onClick(e.getPoint());
          } else if (SwingUtilities.isRightMouseButton(e)) {
            closeShape();
          }
        }

        @Override
        public void mouseMoved(MouseEvent e) {
          if (image != null) {
            double[] f = toField(e.getPoint());
            status.setText(String.format("cursor: (%.2f, %.2f) m    |    %d shapes, %d points",
                f[0], f[1], shapes.size(), current.size()));
          }
        }
      };
      addMouseListener(m);
      addMouseMotionListener(m);
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      if (image == null) {
        return;
      }
      Graphics2D g2 = (Graphics2D) g;
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
      g2.drawImage(image, 0, 0, null);

      for (List<Point> shape : shapes) {
        drawShape(g2, shape, true);
      }
      drawShape(g2, current, false);
    }

    private void drawShape(Graphics2D g2, List<Point> shape, boolean closed) {
      if (shape.isEmpty()) {
        return;
      }
      int[] xs = new int[shape.size()];
      int[] ys = new int[shape.size()];
      for (int i = 0; i < shape.size(); i++) {
        xs[i] = shape.get(i).x;
        ys[i] = shape.get(i).y;
      }
      if (closed) {
        g2.setColor(new Color(255, 60, 60, 70));
        g2.fillPolygon(xs, ys, shape.size());
        g2.setColor(new Color(255, 60, 60));
        g2.drawPolygon(xs, ys, shape.size());
      } else {
        g2.setColor(Color.YELLOW);
        g2.drawPolyline(xs, ys, shape.size());
      }
      for (int i = 0; i < shape.size(); i++) {
        g2.setColor(i == 0 && !closed ? Color.GREEN : (closed ? new Color(255, 60, 60) : Color.YELLOW));
        g2.fillOval(shape.get(i).x - 4, shape.get(i).y - 4, 8, 8);
      }
    }
  }

  public static void main(String[] args) {
    SwingUtilities.invokeLater(() -> new FieldObstacleEditor().setVisible(true));
  }
}
