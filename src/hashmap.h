#ifndef HASHMAP_H
#define HASHMAP_H

template<class V> class Hashmap
{

private:
    int size;

    /**
      * @brief The Node class
      */
     class Node {
        public:
            int hash;
            char* key;
            V* value;
            Node* next;

            Node():hash(0), key("0"), value(0) {
                this->next = 0;
            }
            Node(int hash, char* key, V* value, Node* next): hash(hash), key(key) {
                this->value = value;
                this->next = next;
            }

             char* getKey()   { return key; }
             V getValue()      { return value; }

            int hashCode() {
                return Hashmap::hashCode(key);
            }

            V setValue(V newValue) {
                V oldValue = value;
                value = newValue;
                return oldValue;
            }

            Node operator=(Node node) {
                key = node.key;
                value = node.value;
                hash = node.hash;
                return *this;
            }

            bool operator==(Node* node) {
               if (this->hash == node->hash && this->value == node->value) {
                   return true;
               }
               return false;
            }

            bool operator != (Node* node) {
                if(this == node)
                    return false;
                return true;
            }

        };


     Node* null_node = new Node();

     /**
      * @brief table
      */
     Node table[16];

     /**
        * @brief hashCode
        * @param key
        * @return
        */
       int hashCode(char* key){
           int h = 0;
           for (int i = 0; key[i] !='\0'; i++) {
               h = 31 * h + key[i]*17;
           }
           return h;
       }

       /**
        * @brief hash
        * @param key
        * @return
        */
       int hash(char* key) {
           int h = hashCode(key);
           h += (h << 15) ^ 0xffffcd7d;
           h ^= (unsigned int)(h >> 10);
           h += (h << 3);
           h ^= (unsigned int)(h >> 6);
           h += (h << 2) + (h << 14);
           return h ^ (unsigned int)(h >> 16);
       }

       /**
        * @brief compare
        * @param a
        * @param b
        * @return
        */
       int compare(char a[],char b[]){
           for(int i=0;a[i]!='\0';i++){
               if(a[i]!=b[i])
                   return 0;
           }
           return 1;
       }


       /**
        * @brief putVal
        * @param hash
        * @param key
        * @param value
        * @return
        */
       V* putVal(int hash, char* key, V* value) {
            Node* p; int n, index;
            n = sizeof(table)/sizeof(*table);
            if (*(p = &table[index = (n - 1) & hash]) == null_node) {
                table[index] = Node(hash, key, value, 0);
            }
            else {
                Node* e; char* k;
               if (p->hash == hash &&
                   ((k = p->key) == key || (key != "0" && compare(key, k))))
                   e = p;
               else{
                   for (int count = 0; ; ++count) {
                       if ((e = p->next) == 0) {
                           p->next = new Node(hash, key, value, 0);
                           break;
                       }
                       if (e->hash == hash &&
                           ((k = e->key) == key || (key != "0" && compare(key, k))))
                           break;
                       p = e;
                   }
               }
               if (e != 0) { // existing mapping for key
                   V* oldValue = e->value;
                   if (oldValue == 0)
                       e->value = value;
                   return oldValue;
               }
            }
            return null_node->value;
       }

       /**
        * @brief getNode
        * @param hash
        * @param key
        * @return
        */
       Node* getNode(int hash, char* key) {
               Node* first; Node* e; int n; char* k;
               if ((n = sizeof(table)/sizeof(*table)) > 0 &&
                   (first = &table[(n - 1) & hash]) != null_node) {
                   if (first->hash == hash && // always check first node
                       ((k = first->key) == key || (key != "0" && compare(key, k))))
                       return first;
                   if ((e = first->next) != 0) {
                       do {
                           if (e->hash == hash &&
                               ((k = e->key) == key || (key != 0 && compare(key, k))))
                               return e;
                       } while ((e = e->next) != 0);
                   }
               }
               return null_node;
           }

public:

       Hashmap() {}

       /**
        * @brief put
        * @param key
        * @param value
        * @return
        */
       V *put(char* key, V* value) {
               return putVal(hash(key), key, value);
           }

       /**
         * @brief get
         * @param key
         * @return
         */
        V* get(char* key) {
               Node* e;
               return (e = getNode(hash(key), key)) == this->null_node ? null_node->value: e->value;
           }

};

#endif // HASHMAP_H
