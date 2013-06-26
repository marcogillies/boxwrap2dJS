import org.jbox2d.util.nonconvex.*;
import org.jbox2d.dynamics.contacts.*;
import org.jbox2d.testbed.*;
import org.jbox2d.collision.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.p5.*;
import org.jbox2d.dynamics.*;
import java.lang.reflect.*;
import java.lang.Float;
import processing.core.PApplet;

class CollisionDetector implements ContactListener
{ 
  PApplet m_applet;
  Method collisionMethod;
  boolean collisionHappened = false;
  public CollisionDetector(Physics physics, PApplet applet)
  {
    m_applet = applet;
    physics.getWorld().setContactListener(this);
    Class c = m_applet.getClass();

    // returns the array of Method objects 
    Method[] myMethods = c.getDeclaredMethods();
    for (int i = 0; i < myMethods.length; i++) {
      if (myMethods[i].getName().equals("collision"))
      {
        collisionMethod = myMethods[i];
        //System.out.println("method = " + myMethods[i].toString());
      }
    }
  }

  public void persist(ContactPoint p)
  {
  }

  public void result(ContactResult r)
  {
    if (collisionHappened)
    {
      Object [] args = {
        r.shape1.getBody(), r.shape2.getBody(), new Float(r.normalImpulse)
      };
      collisionMethod.setAccessible(true);
      try {
        collisionMethod.invoke(m_applet, args);
      } 
      catch (IllegalAccessException e) {
        System.out.println("Error invoking method " + collisionMethod.getName());
        e.printStackTrace();
      }
      catch (InvocationTargetException e) {
        System.out.println("Error invoking method " + collisionMethod.getName());
        e.printStackTrace();
      } 
      catch (NullPointerException e) {
        System.out.println("Error invoking method " + collisionMethod.getName());
        e.printStackTrace();
      }
    }
    collisionHappened = false;
  }

  public void add(ContactPoint p)
  {
    collisionHappened = true;
  }

  public void remove(ContactPoint p)
  {
  }
}

