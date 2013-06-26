/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 Mick Grierson, Matthew Yee-King, Marco Gillies
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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

